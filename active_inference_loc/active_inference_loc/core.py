import numpy as np
import time
from nav2_msgs.msg import Particle, ParticleCloud
import scipy
from .utils import ParticleClusturer, ACTION_EFFECTS_long, ACTION_EFFECTS_short, get_map_metadata, get_proximity_risk, calculate_shannon_entropy, calculate_convergence, calculate_spatial_entropy, is_pose_in_collision, calculate_bimodality_position
from .models import predict_motion, predict_motion_batch, raycast_scan_numba, compute_fisher_from_scan_numba
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Pose, Quaternion, Point
import math

class ActiveInferenceController:
    """
    Pure algorithm controller. No ROS dependencies.
    Handles decision logic for all control modes.
    """
    
    def __init__(self, logger, algo_mode, seconds_per_step):
        self.logger = logger
        self.algo_mode = algo_mode
        self.seconds_per_step = seconds_per_step
        self.metrics_pub = None
        self.particle_data_pub = None
        self.status_pub = None
        self.clusturer = ParticleClusturer()
        
        self.time_delta = seconds_per_step # parameter coming from launch init, usually 1 or 5
        self.time_delta_sim = seconds_per_step - 0.1 # time for action and inference, 0.1 seconds for calculations in between
        max_gap = 1.0 # in seconds, how often collision is checked
        num_steps = math.ceil(self.time_delta_sim / max_gap)
        self.collision_checkpoints = np.linspace(self.time_delta_sim/num_steps, self.time_delta_sim, num_steps).astype(np.float64)        

        self.map_2d = None
        self.dist_map = None
        self.map_metadata = None
        self.fisher_map = None # used for D-optimality algo
        self.current_particles = None
        self.current_weights = None

        # Used for Data analysis
        self.efe_epis_particles = None
        self.efe_epis_weights = None
        self.efe_variances = None
        if seconds_per_step<3:
            self.actions_dict = ACTION_EFFECTS_short
        else:
            self.actions_dict = ACTION_EFFECTS_long
        self.shannon_entropy = None
        self.shannon_entropy_norm = None
        self.effective_sample_size_percent = None
        self.runtime_counter = 0
        self.max_runtime = 300 # 5 Minutes max
        self.convergence_parameter = 100
        self.planning_sigma = 0.7   # A value between 0.5 and 1.0 is usually 'reasonable' for planning.
        self.spatial_entropy_res = 0.25 # 25cm bins

        self.wait_streak = 0
        self.clock = None

        self.convergence_threshold = 0.35
        self.bimodal_score_threshold = 0.30 # only location, not rotation 
        self.success_counter = 0 # counts how many successes in localization (convergence and non bimodal) have happened, if 3 in a row the run ends with a Success

        # --- TUNABLE PARAMETERS ---
        self.alpha_epistemic = 1000.0 
        self.beta_pragmatic = 200.0 
        self.risk_penalty_factor = 5.0

        # --- LIDAR PARAMETERS --- should align with lidar from robot (besides num_beams for computation feasibility)
        self.num_beams  = 16            # number of raycast beams, is different from robot lidar! computationally doable with 16
        min_angle        = -1.57   # radians (-90°)
        max_angle        =  1.57   # radians (+90°)
        self.fov_deg = 180.0
        self.laser_min_range = 0.15
        self.laser_max_range = 8.0
        self.laser_angles = np.linspace(min_angle, max_angle, self.num_beams)
        self.laser_std_dev = 0.025

        # --- GROUND TRUTH FOR LOGGING (set by AICNode) ---
        # These are strictly for logging/analysis, NOT for decision-making
        self.ground_truth_pose = None   # [x, y, yaw] from ground truth (for reference only)
        self.starting_pose = None   # [x, y, yaw] initial spawn position
        self.actual_real_position = None # gt + starting pose (x,y)
        self.actual_real_yaw = None # (z)
        self.estimated_position = None  # Estimated position (x, y) from belief state (for logging and error calculation)
        self.position_error = None  # Now: distance from spawn to AMCL estimate
        self.rotational_error = None   # Absolute yaw error between estimated and actual real yaw (which is spawn_yaw + ground_truth_yaw)
        
        # --- METRICS FOR EXPERIMENT LOGGING ---
        self.chosen_action = None
        self.last_raw_epistemic = None
        self.last_raw_pragmatic = None
        self.bimodal_score = None
        self.is_bimodal = None

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

    def update_belief(self, points, weights):
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
        
        # **Compute estimated position (weighted mean)**
        self.estimated_position = np.average(points[:, :2], axis=0, weights=weights)
        self.estimated_rotation = np.arctan2(
            np.average(points[:, 3], weights=weights), # Already sin(yaw)
            np.average(points[:, 2], weights=weights)  # Already cos(yaw)
        )
        
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
            self.actual_real_position = actual_real_position
            
            # Yaw is additive (angles add)
            actual_real_yaw = self.starting_pose[2] + self.ground_truth_pose[2]
            actual_real_yaw = (actual_real_yaw + np.pi) % (2 * np.pi) - np.pi # Standard wrap
            self.actual_real_yaw = actual_real_yaw
            
            # Calculate errors
            self.position_error = np.linalg.norm(self.estimated_position - actual_real_position)
            rotational_error = abs((self.estimated_rotation - actual_real_yaw + np.pi) % (2 * np.pi) - np.pi)            
            self.rotational_error = rotational_error
            
            # Logging
            # self.get_logger().info(f"Starting Position: {self.starting_pose[:2]}, Ground Truth Position: {self.ground_truth_pose[:2]}, Actual Real Position: {actual_real_position}")
            # self.get_logger().info(f"Estimated Position: {self.estimated_position}, Actual Real Position: {actual_real_position}, Positional Error: {self.position_error:.2f}")
            # self.get_logger().info(f"Starting Yaw: {self.starting_pose[2]:.2f}, Ground Truth Yaw: {self.ground_truth_pose[2]:.2f}, Actual Real Yaw: {actual_real_yaw:.2f}")
            # self.get_logger().info(f"Estimated Yaw (raw): {self.estimated_rotation:.2f}, Estimated Yaw (map frame): {self.estimated_rotation_map_frame:.2f}, Rotational Error: {self.rotational_error:.2f}")
            
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
        if self.algo_mode == "active_inf_5":
            return self._run_active_inference(n_raycast_particles=5)
        elif self.algo_mode == "active_inf_500":
            return self._run_active_inference(n_raycast_particles=500)
        elif self.algo_mode == "active_inf_5_h3":
            return self._run_active_inference(n_raycast_particles=5, n_time_horizon=3)
        elif self.algo_mode == "random_walk":
            return self._run_random_walk()
        elif self.algo_mode == "entropy_min":
            return self._run_active_inference(n_raycast_particles=5, only_epistemic=True)
        elif self.algo_mode == "d_opt_particle":
            return self._run_d_opt_particle()
        else:
            self.get_logger().warn(f"Unknown mode '{self.algo_mode}'")
            self.publish_status(f"FAILURE: Unknown mode '{self.algo_mode}'")
            return "WAIT"

    def _run_active_inference(self, n_raycast_particles=5, n_time_horizon=1, only_epistemic=False):  
        """Active Inference with expected free energy.
        Input: 
        - n_raycast_particles: Number of sampled particles (5, 50, 500) for epistemic. pragmatic always 500
            - if == 5: use GMM, else use Importance Sampling
        - n_time_horizon: Number of time horizon into the future (1 or 3)
        - only_epistemic: if Yes, the pragmatic value is ignored, so 0 for each action
        """
        start_time = time.time()

        # --- PHASE 1: UPDATE BELIEF & CHECK STATUS --- 
        gmm_poses, gmm_weights, gmm_variances = self.update_belief_metrics()
        self.efe_variances = gmm_variances

        
        # Bimodality Analysis - for analysis of behavior in 2 hypotheses scenario (only works for GMM)
        self.bimodal_score, self.is_bimodal = calculate_bimodality_position(gmm_poses, gmm_weights)
        
        termination_action = self.check_termination_conditions()
        if termination_action:
            return termination_action
    
        # --- PHASE 2: ALGORITHM SPECIFIC LOGIC ---
        initial_particles = np.copy(self.current_particles)
        initial_weights = np.copy(self.current_weights)
        num_total_p = len(initial_particles)

        # EPISTEMIC
        # check for n_raycast_particles to determine particle n and weights n for epistemic
        # If n_raycast_particles is > 10, use Importance Sampling
        # else GMM with gmm_poses and gmm_weights from above
        sample_size_epistemic = min(num_total_p, n_raycast_particles)
        if sample_size_epistemic > 10:
            p_distribution = initial_weights / np.sum(initial_weights)
            sample_indices_epistemic = np.random.choice(num_total_p, sample_size_epistemic, replace=True, p=p_distribution)
            particles_epistemic = initial_particles[sample_indices_epistemic]
            weights_epistemic = np.ones(sample_size_epistemic) / sample_size_epistemic
            
        else:
            particles_epistemic = np.copy(gmm_poses)
            weights_epistemic = np.copy(gmm_weights)  
           
        if len(particles_epistemic)!=len(weights_epistemic):
            self.get_logger().warn("Particles and Weights not same length! Skipping this run by choosing WAIT action")
            return "WAIT"
        
        self.efe_epis_particles = particles_epistemic
        self.efe_epis_weights = weights_epistemic

        # PRAGMATIC
        # Importance Sampling always with 500 (max) 
        # With Time horizon use GMM particles for pragmatic as well
        if n_time_horizon == 1:
            sample_size_pragmatic = min(num_total_p, 500)
            p_dist_pragmatic = initial_weights / np.sum(initial_weights)
            
            sample_indices_pragmatic = np.random.choice(
                num_total_p, sample_size_pragmatic, replace=True, p=p_dist_pragmatic
            )
            particles_pragmatic = initial_particles[sample_indices_pragmatic]
            weights_pragmatic = np.ones(sample_size_pragmatic) / sample_size_pragmatic
            
            initial_poses_pragmatic = np.copy(particles_pragmatic)
            initial_weights_pragmatic = np.copy(weights_pragmatic)
        else:
            initial_poses_pragmatic = np.copy(particles_epistemic)
            initial_weights_pragmatic = np.copy(weights_epistemic)

        actions = list(self.actions_dict.keys())

        # ── EFE evaluation — single step or horizon tree ─────────────
        def evaluate_efe_single(particles_ep, weights_ep, action):
            """Compute EFE for one action at one step."""
            pred_ep = predict_motion_batch(particles_ep, action, self.actions_dict, dt=self.time_delta_sim)
            raw_epistemic = self.calculate_efe_epistemic(pred_ep, weights_ep)

            if only_epistemic:
                raw_pragmatic = 0.0
            else:
                raw_pragmatic = 0.0
                for checkpoint in self.collision_checkpoints:
                    pred_pr = predict_motion_batch(initial_poses_pragmatic, action, self.actions_dict, dt=checkpoint)
                    checkpoint_risk = self.calculate_efe_pragmatic(pred_pr, initial_weights_pragmatic)
                    raw_pragmatic = max(raw_pragmatic, checkpoint_risk)

            return (self.alpha_epistemic * raw_epistemic) + (self.beta_pragmatic * raw_pragmatic), \
                self.alpha_epistemic * raw_epistemic, \
                self.beta_pragmatic * raw_pragmatic
        
        gamma = 0.9  # discount factor for future EFE steps
        
        def horizon_search(particles_ep, weights_ep, particles_pr, weights_pr, depth):
            action_efes = {}
            for action in actions:

                # ── Epistemic (5 GMM clusters) ────────────────────────
                pred_ep = predict_motion_batch(particles_ep, action, self.actions_dict, dt=self.time_delta_sim)
                raw_epistemic = self.calculate_efe_epistemic(pred_ep, weights_ep)

                # ── Pragmatic (500 particles, propagated correctly) ───
                if only_epistemic:
                    raw_pragmatic = 0.0
                    pred_pr = None
                else:
                    raw_pragmatic = 0.0
                    # for each value in self.collision_checkpoints, i need to predict the new poses, and give this new array to caluclate pragmatic value divided by len(self.collision_checkpoints)
                    for checkpoint in self.collision_checkpoints:
                        pred_pr = predict_motion_batch(particles_pr, action, self.actions_dict, dt=checkpoint)
                        checkpoint_risk = self.calculate_efe_pragmatic(pred_pr, weights_pr)
                        raw_pragmatic = max(raw_pragmatic, checkpoint_risk)

                efe_now = (self.alpha_epistemic * raw_epistemic) + (self.beta_pragmatic * raw_pragmatic)

                if depth > 1:
                    # Reuse pred_ep and pred_pr — no recomputation needed
                    future_efes = horizon_search(
                        pred_ep, weights_ep.copy(),
                        pred_pr if pred_pr is not None else particles_pr,
                        weights_pr.copy(),
                        depth - 1
                    )
                    best_future = min(future_efes.values())
                else:
                    best_future = 0.0

                action_efes[action] = efe_now + gamma * best_future
                #self.get_logger().info(f"For Action {action} in horizon search def: efe_now = {efe_now}. best_future = {best_future}")
            return action_efes
                
        # ── Run search ───────────────────────────────────────────────
        if n_time_horizon == 1:
            # Original single-step logic — no tree overhead
            efe_scores = {}
            details    = {}
            for action in actions:
                total_efe, ep, pr       = evaluate_efe_single(particles_epistemic, weights_epistemic, action)
                efe_scores[action]      = total_efe
                details[action]         = {'epistemic': ep, 'pragmatic': pr}
                # self.get_logger().info("Single Time Horizon")
                # self.get_logger().info(f"Action: {action} | EFE: {total_efe:.2f} (Epistemic: {ep:.2f}, Pragmatic: {pr:.2f})")

        else:
            # Horizon tree search — GMM clusters only
            efe_scores = horizon_search(particles_epistemic, weights_epistemic, 
                                        initial_poses_pragmatic, initial_weights_pragmatic, n_time_horizon)
            # Recompute details for the best action at depth=1 for logging
            details = {}
            for action in actions:
                _, ep, pr          = evaluate_efe_single(particles_epistemic, weights_epistemic, action)
                details[action]    = {'epistemic': ep, 'pragmatic': pr}
                # self.get_logger().info(f"Final EFE For Action {action}: ep:{ep} and pr:{pr}")
        self.get_logger().info(f"Details for all actions: {details}")
        best_action = min(efe_scores, key=efe_scores.get)
        best_detail = details[best_action]

        # --- PHASE 3: FINALIZE & PUBLISH ---
        # Prevent WAIT from being chosen more than 2 times in a row
        best_action = self.handle_wait_streak(best_action, efe_scores, actions)

        # Store for logging
        self.runtime_counter += 1
        self.chosen_action = best_action
        self.last_raw_epistemic = best_detail['epistemic']
        self.last_raw_pragmatic = best_detail['pragmatic']

        self.publish_metrics(best_action, efe_scores, best_detail, gmm_poses, gmm_weights)
        
        total_time = time.time() - start_time
        # self.get_logger().warn(f"Total Time AIC Calculation (efe evaluation): {total_time:.2f}s")
        # self.get_logger().info(f"Final EFE Scores: {efe_scores}. Time for efe calc is {total_time}")
        if total_time > self.time_delta:
            self.get_logger().warn(f"Slowdown: {total_time:.2f}s. Was higher than time_delta of {self.time_delta}")
        
        return best_action
    
    def calculate_efe_epistemic(self, predicted_poses, rep_weights):
        """Calculate expected free energy for epistemic (information gain) value."""
        # 1. Extract metadata for the Numba function
        res = float(self.map_metadata['resolution'])
        ox = float(self.map_metadata['origin_x'])
        oy = float(self.map_metadata['origin_y'])
        pred_scans = raycast_scan_numba(
            poses_4d=np.asarray(predicted_poses, dtype=np.float64),
            dist_map=self.dist_map,res=res,ox=ox,oy=oy,
            fov_deg=self.fov_deg, num_beams=self.num_beams, max_range=self.laser_max_range, min_range=self.laser_min_range, stddev=self.laser_std_dev)

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

    def _run_random_walk(self): 
        """Random action selection."""
        start_time = time.time()
        
        # --- PHASE 1: UPDATE BELIEF & CHECK STATUS --- 
        gmm_poses, gmm_weights, gmm_variances = self.update_belief_metrics()
        self.efe_variances = gmm_variances
        
        # Bimodality Analysis - for analysis of behavior in 2 hypotheses scenario (only works for GMM)
        self.bimodal_score, self.is_bimodal = calculate_bimodality_position(gmm_poses, gmm_weights)
        
        termination_action = self.check_termination_conditions()
        if termination_action:
            return termination_action
        
        # --- PHASE 2: ALGORITHM SPECIFIC LOGIC ---
        available_actions = list(self.actions_dict.keys())
        
        # --- PHASE 2.1: COLLISION AVOIDANCE
        # Check if we have the actual position available
        if self.actual_real_position is not None and self.actual_real_yaw is not None:
            current_pose_4d = np.array([
                self.actual_real_position[0], self.actual_real_position[1],
                np.cos(self.actual_real_yaw), np.sin(self.actual_real_yaw)
                ])

            # HARD CONSTRAINT: Filter collision-causing actions
            safe_actions = []
            for action in available_actions:
                
                # Predict resulting pose from this action
                action_is_safe = True
                for checkpoint in self.collision_checkpoints:
                    predicted_pose = predict_motion(current_pose_4d, action, self.actions_dict, dt=checkpoint)
                    pred_xyz = [
                        predicted_pose[0],
                        predicted_pose[1],
                        np.arctan2(predicted_pose[3], predicted_pose[2])
                    ]
                    if is_pose_in_collision(pred_xyz, self.map_metadata, self.dist_map):
                        action_is_safe = False
                        break                          
                if action_is_safe:
                    safe_actions.append(action)
            # self.get_logger().info(f"Collision filter: {len(safe_actions)}/{len(available_actions)} safe")
        else:
            # Position not available yet, skip collision checking
            self.get_logger().warn("Actual position not available, using all actions")
            safe_actions = available_actions
        
        if not safe_actions:
            self.get_logger().warn(
                "All actions unsafe including WAIT — robot may already be in collision. "
                "Defaulting to WAIT."
            )
            safe_actions = ['WAIT']
        
        # --- PHASE 2.2: ACTION EVALUATION
        best_action = np.random.choice(safe_actions)
        # Random walk has no "EFE components", so we set them to neutral for the CSV/Logs
        efe_scores = {action: 0.0 for action in available_actions}
        best_detail = {'epistemic': -1.0, 'pragmatic': -1.0}

        # --- PHASE 3: FINALIZE & PUBLISH ---
        # Prevent WAIT from being chosen more than 2 times in a row
        best_action = self.handle_wait_streak(best_action, efe_scores, safe_actions=safe_actions)
        self.get_logger().info(f"Random Walk: {best_action}")


        # Store for logging
        self.runtime_counter += 1
        self.chosen_action = best_action
        self.last_raw_epistemic = best_detail['epistemic']
        self.last_raw_pragmatic = best_detail['pragmatic']

        self.publish_metrics(best_action, efe_scores, best_detail, gmm_poses, gmm_weights)
        
        total_time = time.time() - start_time
        if total_time > self.time_delta:
            self.get_logger().warn(f"Slowdown in Random Walk: {total_time:.2f}s. Was higher than time_delta of {self.time_delta}")
        
        return best_action
    
    def _run_d_opt_particle(self):
        start_time = time.time()
        
        # --- PHASE 1: UPDATE BELIEF & CHECK STATUS --- 
        gmm_poses, gmm_weights, gmm_variances = self.update_belief_metrics()
        self.efe_variances = gmm_variances
        
        # Bimodality Analysis - for analysis of behavior in 2 hypotheses scenario (only works for GMM)
        self.bimodal_score, self.is_bimodal = calculate_bimodality_position(gmm_poses, gmm_weights)
        
        termination_action = self.check_termination_conditions()
        if termination_action:
            return termination_action
    
        # --- PHASE 2: ALGORITHM SPECIFIC LOGIC ---
        available_actions = list(self.actions_dict.keys())
        
        # --- PHASE 2.1: COLLISION AVOIDANCE        
        if self.actual_real_position is not None and self.actual_real_yaw is not None:
            current_pose_4d = np.array([
                self.actual_real_position[0], self.actual_real_position[1],
                np.cos(self.actual_real_yaw), np.sin(self.actual_real_yaw)
                ])

            # HARD CONSTRAINT: Filter collision-causing actions
            safe_actions = []
            for action in available_actions:
                
                # Predict resulting pose from this action
                action_is_safe = True
                for checkpoint in self.collision_checkpoints:
                    predicted_pose = predict_motion(current_pose_4d, action, self.actions_dict, dt=checkpoint)
                    pred_xyz = [
                        predicted_pose[0],
                        predicted_pose[1],
                        np.arctan2(predicted_pose[3], predicted_pose[2])
                    ]
                    if is_pose_in_collision(pred_xyz, self.map_metadata, self.dist_map):
                        action_is_safe = False
                        break                          
                if not action_is_safe:
                    continue
                safe_actions.append(action)

        else:
            # Position not available yet, skip collision checking
            self.get_logger().warn("Actual position not available, using all actions")
            safe_actions = available_actions
        
        # --- PHASE 2.2: ACTION EVALUATION: SCORING according to d_opt
        # Scoring uses GMM only — same as epistemic in active inference
        scoring_particles = np.copy(gmm_poses)
        scoring_weights   = np.copy(gmm_weights)
        res          = float(self.map_metadata['resolution'])
        ox           = float(self.map_metadata['origin_x'])
        oy           = float(self.map_metadata['origin_y'])
        map_width_m  = self.map_metadata['width']  * self.map_metadata['resolution']
        map_height_m = self.map_metadata['height'] * self.map_metadata['resolution']
        yaw_scale    = 0.5 * max(map_width_m, map_height_m)
        S            = np.diag([1.0, 1.0, yaw_scale])
        action_scores = {}

        if not safe_actions:
            self.get_logger().warn("No safe actions found! Return Wait.")
            return "WAIT"
        
        for action in safe_actions:
            try:
                # 1. Predict all particles
                pred_particles = predict_motion_batch(scoring_particles, action, self.actions_dict, dt=self.time_delta_sim)
       
                # 2. Raycast all particles
                pred_scans = raycast_scan_numba(
                        poses_4d=pred_particles, dist_map=self.dist_map,res=res, ox=ox, oy=oy,
                        fov_deg=self.fov_deg, num_beams=self.num_beams, max_range=self.laser_max_range, min_range=self.laser_min_range, stddev=self.laser_std_dev
                        )
                
                # 3. Calculate Expected Fisher Information
                fisher_sum = np.zeros((3, 3))
                for i in range(len(pred_particles)):
                    w          = scoring_weights[i]
                    pred_p     = pred_particles[i]

                    pred_p_xyz = [pred_p[0], pred_p[1], np.arctan2(pred_p[3], pred_p[2])]                    
                    F          = compute_fisher_from_scan_numba(
                        pred_p_xyz, 
                        pred_scans[i], 
                        self.laser_angles, 
                        self.laser_max_range
                    )
                    fisher_sum += w * F
                
                # 4. Scale and Score
                fisher_scaled = S @ fisher_sum @ S
                score = float(np.linalg.det(fisher_scaled))
                action_scores[action] = max(score, 0.0)  # det should never be negative for valid FIM
                            
            except (IndexError, Exception) as e:
                self.get_logger().warn(f"Scoring run_d_opt failed for {action}: {e}")
                action_scores[action] = -1e9
        
        if not action_scores:
            self.get_logger().warn("No scoreable actions. Defaulting to WAIT.")
            return "WAIT"
        
        # Select the action with the highest score from safe actions (action_scores only safe_actions)
        best_action = max(action_scores, key=action_scores.get)

        # Prepare metrics for logging (matching AIC structure)
        efe_scores = {a: action_scores.get(a, 0.0) for a in available_actions}
        best_detail = {'epistemic': action_scores[best_action], 'pragmatic': 0.0}
        
        # --- PHASE 3: FINALIZE & PUBLISH ---
        best_action = self.handle_wait_streak(best_action, efe_scores, safe_actions=safe_actions)
        self.get_logger().info(f"D-Optimality: {best_action}")
        self.get_logger().info(f"Full scores: {efe_scores}")

        self.runtime_counter += 1
        self.chosen_action = best_action
        self.last_raw_epistemic = best_detail['epistemic']
        self.last_raw_pragmatic = best_detail['pragmatic']

        self.publish_metrics(best_action, efe_scores, best_detail, gmm_poses, gmm_weights)
        
        total_time = time.time() - start_time
        if total_time > self.time_delta:
            self.get_logger().warn(f"Slowdown in D-Opt: {total_time:.2f}s")
            
        return best_action
    
    def handle_wait_streak(self, best_action, efe_scores, safe_actions):
        """
        If WAIT is chosen 3 times in a row, pick an alternative action.
        Input: 
            - best_action: The action with the lowest score for the current decision step
            - efe_scores: dict of action to EFE score
            - safe_actions: list of collision-free actions
        Output: action string (potentially overridden)
        """
        if best_action == "WAIT":
            self.wait_streak += 1
            if self.wait_streak > 2:
                if self.algo_mode != "random_walk":  
                    # For non-random control all available actions are basically safe, therefore ignore safe_actions variable
                    sorted_actions = sorted(efe_scores, key=efe_scores.get)
                    second_best_action = sorted_actions[1]
                    self.get_logger().info("WAIT chosen > 2 times. Using second-best action.")
                    self.wait_streak = 0
                    return second_best_action
                else: 
                    # For random walk, we have to choose a random new action from the safe_actions list
                    safe_alternatives = [a for a in safe_actions if a != "WAIT"]
                
                    if len(safe_alternatives) > 0:
                        random_action = np.random.choice(safe_alternatives)
                        self.get_logger().info(f"Random Walk: Choosing safe random action instead of WAIT: {random_action}")
                        self.wait_streak = 0
                        return random_action
                    else:
                        self.get_logger().warn("No safe alternatives to WAIT! Robot trapped.")
                        return "WAIT"  # Keep WAIT if truly trapped
        else:
            self.wait_streak = 0
        
        return best_action
    
    def update_belief_metrics(self):
        """
        Calculates and updates standard belief metrics (Entropy & Convergence).
        Always computed from ALL particles for fair comparison
        Returns:
            tuple: (representative_poses, rep_weights) for use by the algo.
        """
        # 1. Shannon Entropy (Statistical Uncertainty)
        self.shannon_entropy = calculate_shannon_entropy(self.current_weights)
        self.spatial_entropy = calculate_spatial_entropy(self.current_particles, self.current_weights, self.spatial_entropy_res)
        
        # 2. Convergence Parameter (The "Stop" Condition)
        self.convergence_parameter = calculate_convergence(self.current_particles, self.current_weights)
        
        # 3. GMM Clustering (Spatial Uncertainty)
        # Clusters for decision making GMM
        gmm_poses, gmm_weights, gmm_variances = self.clusturer.get_representative_clusters_from_gmm(
            self.current_particles, self.current_weights
        )

        return gmm_poses, gmm_weights, gmm_variances
    
    def check_termination_conditions(self):
        """
        Checks if the run should end due to convergence, crash or timeout.
        Returns:
            str: 'WAIT' if a condition is met, otherwise None.
        """
        # Check 1: Convergence
        if self.convergence_parameter < self.convergence_threshold and self.bimodal_score < self.bimodal_score_threshold:
            self.success_counter +=1
            self.get_logger().info(f"Convergence and Non-binomal achieved. Count: {self.success_counter}")
        else:
            self.success_counter = 0

        if self.success_counter >= 3: # Must stay converged for 3 consecutive steps
            self.get_logger().info( f"!!! SUCCESS: CONVERGENCE REACHED 3 times in a row!")
            self.publish_status("SUCCESS: Convergence reached")
            return "WAIT"

        # Check 2: Crash 
        if is_pose_in_collision(self.actual_real_position, self.map_metadata, self.dist_map):
            self.get_logger().info(f"!!! COLLISION at {self.actual_real_position}")
            self.publish_status("FAILURE: Collision")
            return "WAIT"

        # Check 3: Max Runtime
        if self.runtime_counter > self.max_runtime:
            self.get_logger().info(f"!!! MAX RUNTIME REACHED ({self.runtime_counter}) !!!")
            self.publish_status("FAILURE: Max runtime exceeded")
            return "WAIT"
            
        return None
    
    def publish_metrics(self, best_action, efe_scores, best_detail, gmm_poses, gmm_weights):
        """
        Publishes all standard metrics to the ROS topic.
        Args:
            best_action (str): The selected action key.
            efe_scores (dict): Dictionary of scores (or placeholders).
            best_detail (dict): {'epistemic': float, 'pragmatic': float}.
            gmm_poses and gmm_weights: coming from the gmm (5) for bimodality analysis
        """

        if self.metrics_pub is None:
            return

        action_float = self.action_to_id.get(best_action, -1.0) if best_action is not None else -1.0
        
        gmm_weights = gmm_weights / (np.sum(gmm_weights) + 1e-12)

        x_weighted_mean_efe_particles = float(np.sum(gmm_weights * gmm_poses[:, 0]))
        y_weighted_mean_efe_particles = float(np.sum(gmm_weights * gmm_poses[:, 1]))
        mean_cos = np.sum(gmm_weights * gmm_poses[:, 2])
        mean_sin = np.sum(gmm_weights * gmm_poses[:, 3])
        yaw_weighted_mean_efe_particles = float(np.arctan2(mean_sin, mean_cos))

        std_x_weighted_efe_particles = float(np.sqrt(
            np.sum(gmm_weights * self.efe_variances[:, 0]) +
            np.sum(gmm_weights * (gmm_poses[:, 0] - x_weighted_mean_efe_particles)**2)
        ))
        std_y_weighted_efe_particles = float(np.sqrt(
            np.sum(gmm_weights * self.efe_variances[:, 1]) +
            np.sum(gmm_weights * (gmm_poses[:, 1] - y_weighted_mean_efe_particles)**2)
        ))
        
        # For Bimodality analysis, coordinates of the two strongest clusters
        # 1. Get the sorted indices (highest weight first)
        idx = np.argsort(gmm_weights)[::-1]

        # 2. Extract the top two cluster vectors
        # m0/m1 shape is [x, y, cos, sin]
        m0 = gmm_poses[idx[0]]
        m1 = gmm_poses[idx[1]] if len(idx) > 1 else m0

        # --- PEAK 1 DATA ---
        peak1_x = float(m0[0])
        peak1_y = float(m0[1])
        peak1_yaw = float(np.arctan2(m0[3], m0[2]))

        # --- PEAK 2 DATA ---
        peak2_x = float(m1[0])
        peak2_y = float(m1[1])
        peak2_yaw = float(np.arctan2(m1[3], m1[2]))

        # --- PEAK DISTANCE ---
        # Euclidean distance between (x, y) of both peaks
        peak_distance = float(np.linalg.norm(m0[:2] - m1[:2]))
        
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
            float(self.actual_real_position[0]), #12
            float(self.actual_real_position[1]), #13
            float(self.actual_real_yaw), #14
            float(x_weighted_mean_efe_particles), #15
            float(y_weighted_mean_efe_particles), #16
            float(yaw_weighted_mean_efe_particles), #17
            float(std_x_weighted_efe_particles), #18
            float(std_y_weighted_efe_particles), #19
            float(self.bimodal_score), #20
            float(self.is_bimodal), #21
            float(peak1_x), #22
            float(peak1_y), #23
            float(peak1_yaw), #24
            float(peak2_x), #25
            float(peak2_y), #26
            float(peak2_yaw), #27
            float(peak_distance), #28
            float(self.convergence_threshold),  #29
            float(self.bimodal_score_threshold), #30
            float(self.planning_sigma),   # 31
            float(self.spatial_entropy_res), #32
        ]

        self.metrics_pub.publish(metrics_msg)
        
        # Optional: Unified Logging
        self.get_logger().info(
            f"Step {self.runtime_counter} | Action: {best_action} | "
            f"Entropy: {self.shannon_entropy:.2f} | Conv: {self.convergence_parameter:.2f} | Bimodality: {self.bimodal_score:.2f}"
        )