import numpy as np
import time
from nav2_msgs.msg import Particle, ParticleCloud
import scipy
from .utils import ParticleClusturer, ACTION_EFFECTS, get_map_metadata, get_proximity_risk, calculate_shannon_entropy, calculate_convergence, calculate_spatial_entropy
from .models import predict_motion, raycast_scan
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Pose, Quaternion, Point
import math

class ActiveInferenceController:
    """
    Pure algorithm controller. No ROS dependencies.
    Handles decision logic for all control modes.
    """
    
    def __init__(self, logger, algo_mode='active_inf'):
        self.logger = logger
        self.algo_mode = algo_mode
        self.metrics_pub = None
        self.particle_data_pub = None
        self.status_pub = None
        self.clusturer = ParticleClusturer()
        self.time_delta = 1.0  
        self.map_2d = None
        self.dist_map = None
        self.map_metadata = None
        self.current_particles = None
        self.current_weights = None
        self.actions_dict = ACTION_EFFECTS
        self.shannon_entropy = None
        self.shannon_entropy_norm = None
        self.effective_sample_size_percent = None
        self.runtime_counter = 0
        self.max_runtime = 300
        self.convergence_parameter = 100
        self.planning_sigma = 0.7   # A value between 0.5 and 1.0 is usually 'reasonable' for planning.
        self.spatial_entropy_res = 0.25 # 25cm bins

        self.wait_streak = 0
        self.clock = None
        self.convergence_threshold = 0.20

        # --- TUNABLE PARAMETERS ---
        self.alpha_epistemic = 1000.0 
        self.beta_pragmatic = 200.0 
        self.risk_penalty_factor = 5.0
        
        # --- GROUND TRUTH FOR LOGGING (set by AICNode) ---
        # These are strictly for logging/analysis, NOT for decision-making
        self.ground_truth_pose = None   # [x, y, yaw] from ground truth (for reference only)
        self.starting_pose = None   # [x, y, yaw] initial spawn position
        self.estimated_position = None  # Estimated position (x, y) from belief state (for logging and error calculation)
        self.position_error = None  # Now: distance from spawn to AMCL estimate
        self.rotational_error = None   # Absolute yaw error between estimated and actual real yaw (which is spawn_yaw + ground_truth_yaw)
        
        # --- METRICS FOR EXPERIMENT LOGGING ---
        self.chosen_action = None
        self.last_raw_epistemic = None
        self.last_raw_pragmatic = None

        self.action_to_id = {
            action: idx for idx, action in enumerate(self.actions_dict.keys())
            }

    
    def set_particle_publisher(self, pub):
        self.particle_data_pub = pub
    
    def get_logger(self):
        return self.logger
    
    def set_metrics_publisher(self, pub):
        self.metrics_pub = pub

    def set_status_publisher(self, pub):
        self.status_pub = pub

    def publish_status(self, status_message):
        """Publish trial status to /trial_status topic"""
        if self.status_pub is not None:
            msg = String()
            msg.data = status_message
            self.status_pub.publish(msg)
            self.get_logger().info(f'Trial Status: {status_message}')
   
    def set_clock(self, clock):
        self.clock = clock

    def set_map(self, map_msg):
        self.map_metadata = get_map_metadata(map_msg)
        self.map_2d = self.map_metadata['data'] 
        self.dist_map = self.map_metadata['distance_map']
        self.get_logger().info("Map (2D and dist_map) updated successfully.")

    def update_belief(self, points, weights, dt=1.0):
        """
        Vectorized update of belief state.
        Also computes estimated position (mean of belief).
        """
        if len(points) != len(weights):
            self.get_logger().error("Points and weights length mismatch!")
            return
            
        # --- VALIDITY FILTERING ---
        if self.map_metadata is not None:
            res = self.map_metadata['resolution']
            org_x = self.map_metadata['origin_x']
            org_y = self.map_metadata['origin_y']
            width = self.map_metadata['width']
            height = self.map_metadata['height']
            map_data = self.map_metadata['data']

            if not isinstance(map_data, np.ndarray):
                map_data = np.array(map_data, dtype=np.int8).reshape((height, width))

            grid_x = ((points[:, 0] - org_x) / res).astype(int)
            grid_y = ((points[:, 1] - org_y) / res).astype(int)

            in_bounds_x = (grid_x >= 0) & (grid_x < width)
            in_bounds_y = (grid_y >= 0) & (grid_y < height)
            in_bounds_mask = in_bounds_x & in_bounds_y
            is_safe_mask = np.zeros(len(points), dtype=bool)
            valid_indices = in_bounds_mask

            if np.any(valid_indices):
                gy = grid_y[valid_indices]
                gx = grid_x[valid_indices]
                occupied_values = map_data[gy, gx]
                is_free = (occupied_values < 50)
                is_safe_mask[valid_indices] = is_free

            if np.any(is_safe_mask):
                points = points[is_safe_mask]
                weights = weights[is_safe_mask]
            else:
                self.get_logger().warn("⚠️ ALL particles in collision! Keeping previous belief.")
                return
        else:
            self.get_logger().warn("ERROR: Map not set. Skipping collision filtering.")

        # Normalize Weights
        w_sum = np.sum(weights)
        if w_sum > 0:
            weights /= w_sum
        else:
            weights[:] = 1.0 / len(weights)

        # Update State
        self.current_particles = points
        self.current_weights = weights
        self.time_delta = dt
        
        # **Compute estimated position (weighted mean)**
        self.estimated_position = np.average(points[:, :2], axis=0, weights=weights)
        self.estimated_rotation = np.arctan2(
            np.average(np.sin(points[:, 2]), weights=weights),
            np.average(np.cos(points[:, 2]), weights=weights)
        )
        # Apply the same 270° CW rotation to match map frame
        # 270° CW = -π/2 radians
        self.estimated_rotation_map_frame = self.estimated_rotation - np.pi/2
        # Normalize to [-π, π]
        self.estimated_rotation_map_frame = (self.estimated_rotation_map_frame + np.pi) % (2 * np.pi) - np.pi

        # **Compute position error: distance from actual real position to AMCL estimate**
        # Actual real position = spawn_pose + true_position (offset from spawn)
        if self.starting_pose is not None and self.ground_truth_pose is not None:
            # Rotate ground_truth position by starting yaw to map frame
            cos_yaw = np.cos(self.starting_pose[2])
            sin_yaw = np.sin(self.starting_pose[2])
            
            # Apply 2D rotation matrix
            rotated_x = self.ground_truth_pose[0] * cos_yaw - self.ground_truth_pose[1] * sin_yaw
            rotated_y = self.ground_truth_pose[0] * sin_yaw + self.ground_truth_pose[1] * cos_yaw
    
            # Now add to starting position
            actual_real_position = self.starting_pose[:2] + np.array([rotated_x, rotated_y])
            
            # Yaw is additive (angles add)
            actual_real_yaw = self.starting_pose[2] + self.ground_truth_pose[2]
            
            # Calculate errors
            self.position_error = np.linalg.norm(self.estimated_position - actual_real_position)
            rotational_error = abs((self.estimated_rotation_map_frame - actual_real_yaw + np.pi) % (2 * np.pi) - np.pi)
            self.rotational_error = rotational_error
            
            # Logging
            self.get_logger().info(f"Starting Position: {self.starting_pose[:2]}, Ground Truth Position: {self.ground_truth_pose[:2]}, Actual Real Position: {actual_real_position}")
            self.get_logger().info(f"Estimated Position: {self.estimated_position}, Actual Real Position: {actual_real_position}, Positional Error: {self.position_error:.2f}")
            self.get_logger().info(f"Starting Yaw: {self.starting_pose[2]:.2f}, Ground Truth Yaw: {self.ground_truth_pose[2]:.2f}, Actual Real Yaw: {actual_real_yaw:.2f}")
            self.get_logger().info(f"Estimated Yaw (raw): {self.estimated_rotation:.2f}, Estimated Yaw (map frame): {self.estimated_rotation_map_frame:.2f}, Rotational Error: {self.rotational_error:.2f}")
        else:
            self.get_logger().warn("Controller Error: Starting Pose or Ground Truth Pose not available")
        
        if self.particle_data_pub is not None:
            self._publish_filtered_data()

    def _publish_filtered_data(self):
        combined = np.column_stack((self.current_particles, self.current_weights)) 
        msg = Float32MultiArray()
        msg.data = combined.flatten().tolist()
        self.particle_data_pub.publish(msg)
            
    def is_ready(self):
        map_ok = self.map_2d is not None and self.dist_map is not None
        part_ok = self.current_particles is not None
        if not map_ok: self.get_logger().debug("Controller NOT ready: Missing Map / Distance Map")
        if not part_ok: self.get_logger().debug("Controller NOT ready: Missing Particles")
        return map_ok and part_ok

    def decide_action(self):
        """Routes to the appropriate control algorithm.
            Returns: action string (e.g., 'FORWARD', 'LEFT', etc.)
        """

        if not self.is_ready():
            return None

        if self.algo_mode == "active_inf":
            return self._run_active_inference()
        elif self.algo_mode == "random_walk":
            return self._run_random_walk()
        elif self.algo_mode == "classical_aml":
            return self._run_classical_aml()
        else:
            self.get_logger().warn(f"Unknown mode '{self.algo_mode}'")
            self.publish_status(f"FAILURE: Unknown mode '{self.algo_mode}'")
            return "WAIT"

    def _run_active_inference(self, n_raycast_particles):  
        """Active Inference with expected free energy."""
        start_time = time.time()

        # --- PHASE 1: UPDATE BELIEF & CHECK STATUS --- 
        gmm_poses, gmm_weights = self.update_belief_metrics()
        
        termination_action = self.check_termination_conditions()
        if termination_action:
            return termination_action
        
        # --- PHASE 2: ALGORITHM SPECIFIC LOGIC ---
        initial_particles = np.copy(self.current_particles)
        initial_weights = np.copy(self.current_weights)
        
        num_total = len(initial_particles)
        sample_size = min(num_total, 200)
        sample_indices = np.random.choice(num_total, sample_size, replace=False)
        
        sample_particles = initial_particles[sample_indices]
        sample_weights = initial_weights[sample_indices]
        sample_weights = np.ones(sample_size) / sample_size
     
        efe_scores = {}
        details = {} 

        initial_poses_gmm = np.copy(gmm_poses)
        initial_poses_pragmatic = np.copy(sample_particles)
        initial_weights_gmm = np.copy(gmm_weights)
        initial_weights_pragmatic = np.copy(sample_weights)

        for action in self.actions_dict.keys():
            actual_duration = self.time_delta - 0.1
            
            pred_clusters = np.array([predict_motion(p, action, self.actions_dict, dt=actual_duration) for p in initial_poses_gmm])
            raw_epistemic = self.calculate_efe_epistemic(pred_clusters, initial_weights_gmm)

            pred_particles = np.array([predict_motion(p, action, self.actions_dict, dt=actual_duration) for p in initial_poses_pragmatic])
            raw_pragmatic = self.calculate_efe_pragmatic(pred_particles, initial_weights_pragmatic)
            
            total_efe = (self.alpha_epistemic * raw_epistemic) + (self.beta_pragmatic * raw_pragmatic)
            efe_scores[action] = total_efe
            details[action] = {'epistemic': self.alpha_epistemic * raw_epistemic, 'pragmatic': self.beta_pragmatic * raw_pragmatic}
            self.get_logger().info(f"Action: {action} | EFE: {total_efe:.2f} (Epistemic: {raw_epistemic:.2f}, Pragmatic: {raw_pragmatic:.2f})")
        
        best_action = min(efe_scores, key=efe_scores.get)
        best_detail = details[best_action]

        # --- PHASE 3: FINALIZE & PUBLISH ---
        # Prevent WAIT from being chosen more than 2 times in a row
        best_action = self.handle_wait_streak(best_action, efe_scores)

        # Store for logging
        self.runtime_counter += 1
        self.chosen_action = best_action
        self.last_raw_epistemic = best_detail['epistemic']
        self.last_raw_pragmatic = best_detail['pragmatic']

        self.publish_metrics(best_action=best_action, efe_scores=efe_scores, best_detail=best_detail)
        
        total_time = time.time() - start_time
        self.get_logger().warn(f"Total Time AIC Calculation (efe evaluation): {total_time:.2f}s")

        if total_time > self.time_delta:
            self.get_logger().warn(f"Slowdown: {total_time:.2f}s")
        
        return best_action
    
    def calculate_efe_epistemic(self, predicted_poses, rep_weights):
        """Calculate expected free energy for epistemic (information gain) value."""
        pred_scans = raycast_scan(predicted_poses, self.dist_map, self.map_metadata)

        # Sanity checks / normalization
        rep_weights = np.asarray(rep_weights, dtype=np.float64)
        if rep_weights.ndim != 1:
            raise ValueError("rep_weights must be a 1D array")
        if pred_scans.shape[0] != len(rep_weights):
            raise ValueError("Number of predicted scans must match number of weights.")

        # Ensure numeric types
        pred_scans = np.asarray(pred_scans, dtype=np.float64)

        # Pairwise squared differences between scan vectors
        diffs = pred_scans[:, np.newaxis, :] - pred_scans[np.newaxis, :, :]
        sq_diffs = np.sum(diffs**2, axis=2)

        # Log-likelihood (up to a constant) under isotropic Gaussian sensor model
        num_beams = pred_scans.shape[1]
        mse = sq_diffs / num_beams
        ll_matrix = -0.5 * mse / (self.planning_sigma**2)

        # Stabilize weights and compute posterior mixture responsibilities
        log_rep_weights = np.log(rep_weights + 1e-12)
        log_w_matrix = ll_matrix + log_rep_weights[np.newaxis, :]
        # LogSumExp for numerical stability
        log_z = scipy.special.logsumexp(log_w_matrix, axis=1, keepdims=True)
        # Exponentiate to get probabilities P(z|x)
        w_matrix = np.exp(log_w_matrix - log_z)

        # Numerically-stable entropy: clip tiny values to avoid negative zeros
        w_clipped = np.clip(w_matrix, 1e-12, 1.0)
        entropies = -np.sum(w_clipped * np.log(w_clipped), axis=1)
        expected_entropy = np.sum(rep_weights * entropies)

        return float(expected_entropy)

    def calculate_efe_pragmatic(self, pred_particles, sample_weights):
        """Calculate expected free energy for pragmatic (safety) value."""
        total_risk = 0.0
        for i in range(len(pred_particles)):
            risk = get_proximity_risk(pred_particles[i], self.map_metadata)
            total_risk += sample_weights[i] * risk
        
        return total_risk * self.risk_penalty_factor

    #TODO: Implement classical AML decision logic if needed. For now, they just return "WAIT" to let external nodes handle them.
    # Make sure to also implement the same requirements as in aic. 
    # Maximal runtime!
    # 
    def _run_random_walk(self):
        """Random action selection."""
        start_time = time.time()
        
        # --- PHASE 1: UPDATE BELIEF & CHECK STATUS --- 
        gmm_poses, gmm_weights = self.update_belief_metrics()
        
        termination_action = self.check_termination_conditions()
        if termination_action:
            return termination_action
        
         # --- PHASE 2: ALGORITHM SPECIFIC LOGIC ---
        available_actions = list(self.actions_dict.keys())
        best_action = np.random.choice(available_actions)
        
        # Random walk has no "EFE components", so we set them to neutral for the CSV/Logs
        efe_scores = {action: 0.0 for action in available_actions}
        best_detail = {'epistemic': 0.0, 'pragmatic': 0.0}

        # --- PHASE 3: FINALIZE & PUBLISH ---
        # Prevent WAIT from being chosen more than 2 times in a row
        best_action = self.handle_wait_streak(best_action, efe_scores)
        self.get_logger().info(f"Random Walk: {best_action}")


        # Store for logging
        self.runtime_counter += 1
        self.chosen_action = best_action
        self.last_raw_epistemic = best_detail['epistemic']
        self.last_raw_pragmatic = best_detail['pragmatic']

        self.publish_metrics(best_action=best_action, efe_scores=efe_scores, best_detail=best_detail)
        
        total_time = time.time() - start_time
        if total_time > self.time_delta:
            self.get_logger().warn(f"Slowdown in Random Walk: {total_time:.2f}s")
        
        return best_action

    def _run_classical_aml(self):
        """Classical AML: update belief but don't take action."""
        self.chosen_action = "WAIT"
        return "WAIT"

    def handle_wait_streak(self, best_action, efe_scores):
        """If WAIT is chosen 3 times in a row, pick the second-best action.
        Input: best_action (string), efe_scores (dict of action to EFE score)
            - best_action: The action with the lowest score for the current decision step.
        Output: action string (potentially overridden)
        """
        if best_action == "WAIT":
            self.wait_streak += 1
            if self.wait_streak > 2:
                if self.algo_mode != "random_walk":  # For random walk, we have to choose a random new action instead of the second-best
                    sorted_actions = sorted(efe_scores, key=efe_scores.get)
                    second_best_action = sorted_actions[1]
                    self.get_logger().info("WAIT chosen > 2 times. Using second-best action.")
                    self.wait_streak = 0
                    return second_best_action
                else: 
                    available_actions = list(self.actions_dict.keys())
                    available_actions.remove("WAIT")
                    random_action = np.random.choice(available_actions)
                    self.get_logger().info("WAIT chosen > 2 times in Random Walk. Choosing random action.")
                    self.wait_streak = 0
                    return random_action
        else:
            self.wait_streak = 0
        
        return best_action
    
    def update_belief_metrics(self):
        """
        Calculates and updates standard belief metrics (Entropy & Convergence).
        Returns:
            tuple: (representative_poses, rep_weights, rep_variances) for use by the algo.
        """
        # 1. Shannon Entropy (Statistical Uncertainty)
        self.shannon_entropy = calculate_shannon_entropy(self.current_weights)
        self.spatial_entropy = calculate_spatial_entropy(self.current_particles, self.current_weights, self.spatial_entropy_res)

        # 2. GMM Clustering (Spatial Uncertainty)
        # We need these clusters for both convergence checking AND decision making
        gmm_poses, gmm_weights, gmm_vars = self.clusturer.get_representative_clusters_from_gmm(
            self.current_particles, self.current_weights
        )
        
        # 3. Convergence Parameter (The "Stop" Condition)
        self.convergence_parameter = calculate_convergence(gmm_poses, gmm_weights, gmm_vars)
        
        return gmm_poses, gmm_weights
    
    def check_termination_conditions(self):
        """
        Checks if the run should end due to convergence or timeout.
        Returns:
            str: 'WAIT' if a condition is met, otherwise None.
        """
        # Check 1: Convergence
        if self.convergence_parameter < self.convergence_threshold:
            self.get_logger().info(f"!!! CONVERGENCE REACHED ({self.convergence_parameter:.3f} < {self.convergence_threshold}) !!!")
            self.publish_status("SUCCESS: Convergence reached")
            return "WAIT"

        # Check 2: Max Runtime
        if self.runtime_counter > self.max_runtime:
            self.get_logger().info(f"!!! MAX RUNTIME REACHED ({self.runtime_counter}) !!!")
            self.publish_status("FAILURE: Max runtime exceeded")
            return "WAIT"
            
        return None
    
    def publish_metrics(self, best_action, efe_scores, best_detail):
        """
        Publishes all standard metrics to the ROS topic.
        Args:
            best_action (str): The selected action key.
            efe_scores (dict): Dictionary of scores (or placeholders).
            best_detail (dict): {'epistemic': float, 'pragmatic': float}.
        """
        if self.metrics_pub is None:
            return

        action_float = self.action_to_id.get(best_action, -1.0) if best_action is not None else -1.0
        # Convert action string to float ID if necessary (or keep as string if your msg supports it)
        # Assuming you have a helper or mapping for action->float, otherwise -1.0
        action_float = -1.0 
        # Example: action_float = self.action_to_id.get(best_action, -1.0)

        metrics_msg = Float32MultiArray()
        metrics_msg.data = [
            float(best_detail.get('epistemic', 0.0)), #0
            float(best_detail.get('pragmatic', 0.0)), #1
            float(efe_scores.get(best_action, 0.0)), #2
            float(self.alpha_epistemic), #3
            float(self.beta_pragmatic), #4
            float(self.runtime_counter), #5
            float(self.convergence_parameter), #6
            float(self.position_error) if self.position_error is not None else -1.0, #7
            float(self.rotational_error) if self.rotational_error is not None else -1.0, #8
            float(self.shannon_entropy) if self.shannon_entropy is not None else -1.0, #9
            float(self.spatial_entropy) if self.spatial_entropy is not None else -1.0, #10
            action_float, #11
        ]
        self.metrics_pub.publish(metrics_msg)
        
        # Optional: Unified Logging
        self.get_logger().info(
            f"Step {self.runtime_counter} | Action: {best_action} | "
            f"Entropy: {self.shannon_entropy:.2f} | Conv: {self.convergence_parameter:.2f}"
        )