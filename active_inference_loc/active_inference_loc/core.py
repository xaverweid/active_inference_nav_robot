import numpy as np
import time
from nav2_msgs.msg import Particle, ParticleCloud
import scipy
from .utils import ParticleClusturer, ACTION_EFFECTS, get_map_metadata, get_proximity_risk, calculate_shannon_entropy, calculate_convergence
from .models import predict_motion, raycast_scan
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Pose, Quaternion, Point
import math

class ActiveInferenceController:
    """
    Pure algorithm controller. No ROS dependencies.
    Handles decision logic for all control modes.
    """
    
    def __init__(self, logger, algo_mode='active_inf', spawn_pose=np.array([0.0, 0.0, 0.0])):
        self.logger = logger
        self.algo_mode = algo_mode
        self.metrics_pub = None
        self.particle_data_pub = None
        self.status_pub = None
        self.clusturer = ParticleClusturer()
        self.time_delta = 1.0  
        self.map_2d = None
        self.map_metadata = None
        self.current_particles = None
        self.current_weights = None
        self.actions_dict = ACTION_EFFECTS
        self.shannon_entropy = None
        self.runtime_counter = 0
        self.max_runtime = 300
        self.convergence_parameter = 100
        self.planning_sigma = 0.7   # A value between 0.5 and 1.0 is usually 'reasonable' for planning.

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
        self.yaw_error = None   # Absolute yaw error between estimated and actual real yaw (which is spawn_yaw + ground_truth_yaw)
        
        # --- METRICS FOR EXPERIMENT LOGGING ---
        self.last_action = None
        self.last_raw_epistemic = None
        self.last_raw_pragmatic = None
    
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
        self.get_logger().info("Map updated successfully.")

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
        
        # **Compute position error: distance from actual real position to AMCL estimate**
        # Actual real position = spawn_pose + true_position (offset from spawn)
        if self.starting_pose is not None and self.ground_truth_pose is not None:
            actual_real_position = self.starting_pose[:2] + self.ground_truth_pose[:2]
            self.get_logger().info(f"Starting Position: {self.starting_pose[:2]}, Ground Truth Position: {self.ground_truth_pose[:2]}, Actual Real Position: {actual_real_position}")
            self.get_logger().info(f"Estimated Position: {self.estimated_position}, Actual Real Position: {actual_real_position}")
            self.position_error = np.linalg.norm(self.estimated_position - actual_real_position)

            actual_real_yaw = self.starting_pose[2] + self.ground_truth_pose[2]
            rotational_error = abs((self.estimated_rotation - actual_real_yaw + np.pi) % (2 * np.pi) - np.pi)
            self.yaw_error = rotational_error
            self.get_logger().info(f"Starting Yaw: {self.starting_pose[2]:.2f}, Ground Truth Yaw: {self.ground_truth_pose[2]:.2f}, Actual Real Yaw: {actual_real_yaw:.2f}")
            self.get_logger().info(f"Estimated Yaw: {self.estimated_rotation:.2f}, Actual Real Yaw: {actual_real_yaw:.2f}, Yaw Error: {self.yaw_error:.2f}")
        
        if self.particle_data_pub is not None:
            self._publish_filtered_data()

    def _publish_filtered_data(self):
        combined = np.column_stack((self.current_particles, self.current_weights)) 
        msg = Float32MultiArray()
        msg.data = combined.flatten().tolist()
        self.particle_data_pub.publish(msg)
            
    def is_ready(self):
        map_ok = self.map_2d is not None
        part_ok = self.current_particles is not None
        if not map_ok: self.get_logger().debug("Controller NOT ready: Missing Map")
        if not part_ok: self.get_logger().debug("Controller NOT ready: Missing Particles")
        return map_ok and part_ok

    def decide_action(self):
        """Routes to the appropriate control algorithm."""
        if not self.is_ready():
            return None

        if self.algo_mode == "active_inf":
            return self._run_active_inference()
        elif self.algo_mode == "random_walk":
            return self._run_random_walk()
        elif self.algo_mode == "passive_amcl":
            return self._run_passive_amcl()
        elif self.algo_mode == "classical_amcl":
            return self._run_classical_amcl()
        elif self.algo_mode == "standard_dwa":
            return self._run_standard_dwa()
        else:
            self.get_logger().warn(f"Unknown mode '{self.algo_mode}'")
            self.publish_status(f"FAILURE: Unknown mode '{self.algo_mode}'")
            return "WAIT"

    def _run_active_inference(self):  
        """Active Inference with expected free energy."""
        start_time = time.time()
        initial_particles = np.copy(self.current_particles)
        initial_weights = np.copy(self.current_weights)
        
        num_total = len(self.current_particles)
        sample_size = min(num_total, 200)
        sample_indices = np.random.choice(num_total, sample_size, replace=False)
        
        sample_particles = self.current_particles[sample_indices]
        sample_weights = self.current_weights[sample_indices]
        sample_weights = np.ones(sample_size) / sample_size
        
        representative_poses_gmm, rep_weights_gmm = self.clusturer.get_representative_clusters_from_gmm(
            initial_particles, initial_weights
        )
     
        self.shannon_entropy = calculate_shannon_entropy(self.current_weights)

        efe_scores = {}
        details = {} 

        initial_poses_gmm = np.copy(representative_poses_gmm)
        initial_poses_pragmatic = np.copy(sample_particles)
        initial_weights_gmm = np.copy(rep_weights_gmm)
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

        if best_action == "WAIT":
            self.wait_streak += 1
            if self.wait_streak > 2:
                sorted_actions = sorted(efe_scores, key=efe_scores.get)
                best_action = sorted_actions[1]
                self.get_logger().info("WAIT chosen > 2 times. Using second-best action.")
        else:
            self.wait_streak = 0

        best_detail = details[best_action]

        # Metrics & Logging
        total_time = time.time() - start_time
        self.runtime_counter += 1
        self.convergence_parameter = calculate_convergence(representative_poses_gmm, rep_weights_gmm)
        
        if self.convergence_parameter < self.convergence_threshold:
            self.get_logger().info("!!! CONVERGENCE REACHED !!!")
            self.publish_status("SUCCESS: Convergence reached")
            return "WAIT"
        if self.runtime_counter > self.max_runtime:
            self.get_logger().info("!!! MAX RUNTIME REACHED !!!")
            self.publish_status("FAILURE: Max runtime exceeded")
            return "WAIT"

        if total_time > self.time_delta:
            self.get_logger().warn(f"AIC Slowdown: {total_time:.2f}s > {self.time_delta}s")

        # Store for logging
        self.last_action = best_action
        self.last_raw_epistemic = best_detail['epistemic']
        self.last_raw_pragmatic = best_detail['pragmatic']

        if self.metrics_pub is not None:
            metrics_msg = Float32MultiArray()
            metrics_msg.data = [
                float(best_detail['epistemic']), 
                float(best_detail['pragmatic']), 
                float(efe_scores[best_action]),
                float(self.alpha_epistemic),
                float(self.beta_pragmatic),
                float(self.runtime_counter),
                float(self.convergence_parameter),
                float(self.position_error) if self.position_error is not None else -1.0,
                float(self.yaw_error) if self.yaw_error is not None else -1.0
            ]
            self.metrics_pub.publish(metrics_msg)

        self.get_logger().info(
            f"Result: [{best_action}] | EFE: {efe_scores[best_action]:.2f} "
            f"| Epistemic: {best_detail['epistemic']:.2f} | Pragmatic: {best_detail['pragmatic']:.2f}"
        )
        
        return best_action
    
    def calculate_efe_epistemic(self, predicted_poses, rep_weights):
        """Calculate expected free energy for epistemic (information gain) value."""
        pred_scans = raycast_scan(predicted_poses, self.map_2d, self.map_metadata)

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

    #TODO: Implement DWA and classical AMCL decision logic if needed. For now, they just return "WAIT" to let external nodes handle them.
    # Make sure to also implement the same requirements as in aic. 
    # f.e.: after 3 WAITs, select the second-best action to avoid getting stuck in local minima.
    # 
    def _run_random_walk(self):
        """Random action selection."""
        action = np.random.choice(list(self.actions_dict.keys()))
        self.get_logger().info(f"Random Walk: {action}")
        self.last_action = action
        return action

    def _run_passive_amcl(self):
        """Passive mode: don't move, let AMCL update."""
        self.last_action = "WAIT"
        return "WAIT"
    
    def _run_classical_amcl(self):
        """Classical AMCL: update belief but don't take action."""
        self.last_action = "WAIT"
        return "WAIT"
    
    def _run_standard_dwa(self):
        """Standard DWA: handled externally."""
        self.last_action = None
        return None
