import numpy as np
import time
from nav2_msgs.msg import Particle, ParticleCloud
import scipy  # Added for performance tracking
from .utils import ParticleClusturer, ACTION_EFFECTS, get_map_metadata, get_proximity_risk, calculate_shannon_entropy, calculate_convergence
from .models import predict_motion, raycast_scan
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose, Quaternion, Point
import math

class ActiveInferenceController:
    def __init__(self, logger):
        self.logger = logger
        self.metrics_pub = None
        self.particle_data_pub = None
        self.clusturer = ParticleClusturer()
        self.time_delta = 1.0  
        self.map_2d = None
        self.map_metadata = None
        self.current_particles = None
        self.current_weights = None
        self.actions_dict = ACTION_EFFECTS
        self.shannon_entropy = None
        self.runtime_counter = 0
        self.convergence_parameter = 0
        self.lidar_sigma = 0.15  # Standard deviation for LiDAR likelihood model
        self.wait_streak = 0
        self.clock = None

        # --- TUNABLE PARAMETERS (The "Personality" of your Robot) ---
        # 1. Epistemic Weight (gamma): Curiosity. 
        # Higher = Robot explores more to reduce uncertainty.
        self.alpha_epistemic = 500.0 

        # 2. Pragmatic Weight (beta): Importance of following the "Goal/Safety".
        # Higher = Robot prioritizes safety/risk avoidanc.
        # Keep beta_pragmatic * risk_penalty_factor = 1000 for having 1000 as maximum risk value
        self.beta_pragmatic = 200.0 

        # 3. Risk Penalty: The "cost" of a single collision.
        # This scales the pragmatic value before it is weighted by beta.
        self.risk_penalty_factor = 5.0
    
    def set_particle_pusblisher(self, pub):
        self.particle_data_pub = pub
    
    def get_logger(self):
        return self.logger
    
    def set_metrics_publisher(self, pub):
        self.metrics_pub = pub

    def set_clock(self, clock):
        self.clock = clock

    def set_map(self, map_msg):
        self.map_metadata = get_map_metadata(map_msg)
    
        # You don't need to recalculate raw_data or reshape here anymore
        # because 'data' in map_metadata is ALREADY the reshaped 2D numpy array.
        self.map_2d = self.map_metadata['data'] 
        
        self.get_logger().info("Map updated successfully.")
        # self.get_logger().info(f"Map metadata: width={self.map_metadata['width']}, height={self.map_metadata['height']}, resolution={self.map_metadata['resolution']}")
        ## Log map stats for debugging
        # obstacles = (self.map_2d >= 50) | (self.map_2d == -1)
        # self.get_logger().info(f"Map stats: Shape {self.map_2d.shape}, Obstacles: {np.sum(obstacles)}, Free: {np.sum(~obstacles)}, Unknown: {np.sum(self.map_2d == -1)}")
        # self.get_logger().info(f"Distance map range: {np.min(self.map_metadata['distance_map']):.3f} - {np.max(self.map_metadata['distance_map']):.3f} m")
    def update_belief(self, points, weights, dt=1.0):
        """
        Vectorized update of belief state.
        - Uses map_metadata dictionary structure.
        - Filters out points that are in collision.
        """

        # 1. Sanity Check
        if len(points) != len(weights):
            self.get_logger.error("Points and weights length mismatch at start!")
            return
            
        # --- NEW: VALIDITY FILTERING ---
        if self.map_metadata is not None:
            
            # --- A. Extract Map Info (From your Dictionary) ---
            res = self.map_metadata['resolution']
            org_x = self.map_metadata['origin_x']
            org_y = self.map_metadata['origin_y']
            width = self.map_metadata['width']
            height = self.map_metadata['height']
            map_data = self.map_metadata['data']

            if not isinstance(map_data, np.ndarray):
                map_data = np.array(map_data, dtype=np.int8).reshape((height, width))

            # --- B. World -> Grid Conversion (Vectorized) ---
            grid_x = ((points[:, 0] - org_x) / res).astype(int)
            grid_y = ((points[:, 1] - org_y) / res).astype(int)

            # --- C. Bounds Check (Vectorized) ---
            # Create a boolean mask of valid points inside the map dimensions
            in_bounds_x = (grid_x >= 0) & (grid_x < width)
            in_bounds_y = (grid_y >= 0) & (grid_y < height)
            in_bounds_mask = in_bounds_x & in_bounds_y

            # --- D. Collision Check (Vectorized) ---
            # Initialize a "safe" mask (default False)
            is_safe_mask = np.zeros(len(points), dtype=bool)

            # (Using map_data[y, x] because your reshape was (Height, Width))
            valid_indices = in_bounds_mask

            # Advanced NumPy Indexing:
            # We look up the map values for all valid points in one shot
            # In ROS maps: 0 = Free, 100 = Occupied, -1 = Unknown/Unexplored
            # You might want to treat -1 as collision too, or just 100.
            # Here I assume we only keep points where map is explicitly 0 (Free).
            if np.any(valid_indices):
                # Extract grid coordinates for valid points
                gy = grid_y[valid_indices]
                gx = grid_x[valid_indices]
                
                # Check values
                occupied_values = map_data[gy, gx]
                
                # Create a mask for free space (value < 50)
                is_free = (occupied_values < 50)
                
                # Update the main mask
                is_safe_mask[valid_indices] = is_free

            # --- E. Apply Filter ---
            if np.any(is_safe_mask):
                points = points[is_safe_mask]
                weights = weights[is_safe_mask]
            else:
                # Thesis Logic: If ALL particles are bad, don't just return. 
                # Expand uncertainty or keep old belief, but warn heavily.
                self.get_logger().warn("⚠️ ALL particles in collision! Keeping previous belief.")
                return
        else:
            self.get_logger().warn("ERROR: Map not set. Skipping collision filtering.")

        # 3. Normalize Weights (Vectorized)
        w_sum = np.sum(weights)
        if w_sum > 0:
            weights /= w_sum
        else:
            weights[:] = 1.0 / len(weights)

        # 4. Update State
        self.current_particles = points
        self.current_weights = weights
        self.time_delta = dt

        if self.particle_data_pub is not None:
            self._publish_filtered_data()

    def _publish_filtered_data(self):
        # points_4d is (N, 4), weights is (N,)
        # We need to make weights (N, 1) to stack them horizontally
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
        if not self.is_ready():
            return None

        start_time = time.time()

        initial_particles = np.copy(self.current_particles)
        initial_weights = np.copy(self.current_weights)
        
        # 1. SAMPLE PARTICLES (For Pragmatic/Risk)
        # Since weights are equal, we take a random uniform sample. 
        # This sample naturally represents the probability density of the cloud.
        num_total = len(self.current_particles)
        sample_size = min(num_total, 200)
        
        # Randomly select indices without replacement
        sample_indices = np.random.choice(num_total, sample_size, replace=False)
        
        # These represent the "possible bodies" of the robot
        sample_particles = self.current_particles[sample_indices]
        sample_weights = self.current_weights[sample_indices]
        
        # Since they are unweighted, we treat them all as equally likely (1/sample_size)
        sample_weights = np.ones(sample_size) / sample_size
         
        # 2. CONDENSE BELIEF (For Epistemic/Info Gain)
        representative_poses_gmm, rep_weights_gmm = self.clusturer.get_representative_clusters_from_gmm(
            initial_particles, initial_weights
        )
        
        self.shannon_entropy = calculate_shannon_entropy(rep_weights_gmm)
        # Log how many clusters we actually found (might be < 5 if particles are tight)
        self.get_logger().info(f"Thinking... (Clustered {len(initial_particles)} particles into {len(representative_poses_gmm)} hypotheses)")

        efe_scores = {}
        details = {} 

        initial_poses_gmm=np.copy(representative_poses_gmm)
        initial_poses_pragmatic=np.copy(sample_particles)
        initial_weights_gmm=np.copy(rep_weights_gmm)
        initial_weights_pragmatic=np.copy(sample_weights)

        # 2. EVALUATE EFE (G)
        for action in self.actions_dict.keys():
            action_start = time.time()

            actual_duration = self.time_delta - 0.1
            
            # Predict trajectory for 5 clusters (Epistemic) and epistemic value
            pred_clusters = np.array([predict_motion(p, action, self.actions_dict, dt=actual_duration) for p in initial_poses_gmm])
            raw_epistemic = self.calculate_efe_epistemic(pred_clusters, initial_weights_gmm)

            

            # Predict trajectory for 200 particles (Pragmatic) and pragmatic value
            pred_particles = np.array([predict_motion(p, action, self.actions_dict, dt=actual_duration) for p in initial_poses_pragmatic])
            raw_pragmatic = self.calculate_efe_pragmatic(pred_particles, initial_weights_pragmatic)
            
            total_efe = (self.alpha_epistemic * raw_epistemic) + (self.beta_pragmatic * raw_pragmatic)
            
            efe_scores[action] = total_efe
            details[action] = {'epistemic': self.alpha_epistemic * raw_epistemic, 'pragmatic': self.beta_pragmatic * raw_pragmatic}
            
            # self.get_logger().debug(f"Action [{action}] evaluated in {time.time()-action_start:.3f}s")
            # self.get_logger().info(f"Action [{action}] scores on epistemic {raw_epistemic*self.alpha_epistemic}, pragmatic {raw_pragmatic*self.beta_pragmatic}")


        # 3. SELECT ACTION
        best_action = min(efe_scores, key=efe_scores.get)

        # Incorporate Deadlock Resolver
        if best_action == "WAIT":
            self.wait_streak += 1
             # If stuck, boost Alpha (Curiosity) exponentially
            if self.wait_streak > 2:
                sorted_actions = sorted(efe_scores, key=efe_scores.get)
                second_best_action = sorted_actions[1]
                best_action = second_best_action
                self.get_logger().info("WAIT was chosen more than 2 times. Second best option was chosen")
        else:
            self.wait_streak = 0

        best_detail = details[best_action]

        # 4. METRICS & LOGGING
        total_time = time.time() - start_time
        self.runtime_counter += 1
        self.convergence_parameter  = calculate_convergence(representative_poses_gmm, rep_weights_gmm)
        if self.convergence_parameter < 0.20:
            self.get_logger().info("!!! CONVERGENCE REACHED !!!")
            return "WAIT"

        
        # Critical warning if the 'thought' took longer than the robot's step
        if total_time > self.time_delta:
            self.get_logger().warn(f"AIC Slowdown: Thought took {total_time:.2f}s, but step is {self.time_delta}s!")


        if self.metrics_pub is not None:
            metrics_msg = Float32MultiArray()
            metrics_msg.data = [
                float(best_detail['epistemic']), 
                float(best_detail['pragmatic']), 
                float(efe_scores[best_action]),
                float(self.alpha_epistemic),
                float(self.beta_pragmatic),
                float(self.runtime_counter),
                float(self.convergence_parameter)
            ]
            self.metrics_pub.publish(metrics_msg)

        # Log the breakdown so you can see WHY it picked that action
        self.get_logger().info(
            f"Result: [{best_action}] | Total EFE: {efe_scores[best_action]:.2f} "
            f"(Expected Entropy Change: {best_detail['epistemic']:.2f}, Risk: {best_detail['pragmatic']:.2f}) | "
            f"Time: {total_time:.2f}s"
        )
        
        return best_action
    
    def calculate_efe_epistemic(self, predicted_poses, rep_weights):
    # (K, B) array of scans
        pred_scans = raycast_scan(predicted_poses, self.map_2d, self.map_metadata)
        if pred_scans.shape[0] != len(rep_weights):
            raise ValueError("Number of predicted scans must match number of weights.")
        
        # 1. Compute ALL-to-ALL Log-Likelihoods in one shot (Vectorized)
        # Resulting shape: (K, K) where entry [j, i] is LL of scan j given pose i
        # We use broadcasting: (K, 1, B) - (1, K, B) -> (K, K, B)
        diffs = pred_scans[:, np.newaxis, :] - pred_scans[np.newaxis, :, :]
        # Sum over the beam dimension (B)
        sq_diffs = np.sum(diffs**2, axis=2)
        
        # Gaussian log-likelihood matrix
        ll_matrix = -0.5 * sq_diffs / (self.lidar_sigma**2)

        # 2. Add log-weights and normalize to get posterior w_ij
        log_rep_weights = np.log(rep_weights + 1e-12)
        # Add weights to each row: log_w_ij[j, i] = log_weights[i] + ll[j, i]
        log_w_matrix = ll_matrix + log_rep_weights[np.newaxis, :]
        
        # LogSumExp across rows to normalize each hypothetical posterior
        log_z = scipy.special.logsumexp(log_w_matrix, axis=1, keepdims=True)
        w_matrix = np.exp(log_w_matrix - log_z)

        # 3. Calculate Shannon Entropy for each hypothetical posterior
        # H = -sum(p * log(p))
        entropies = -np.sum(w_matrix * np.log(w_matrix + 1e-12), axis=1)

        # 4. Expected Entropy: Sum(P(z_j) * H_j)
        # We use rep_weights as the probability of encountering measurement j
        expected_entropy = np.sum(rep_weights * entropies)

        return float(expected_entropy)

    def calculate_efe_pragmatic(self, pred_particles, sample_weights):
        """
        Receives predicted poses for ALL particles.
        The weights are the same for each pose (comes from ParticleCloud, no weights given)
        Returns the expected risk (weighted average) for 1 specific action.
        total_risk should return a value between 0 and 1 before weighting by beta and risk_penalty_factor.
        
        G_pragmatic = sum( weight_i * risk(pose_i) )
    
        """
        total_risk = 0.0

        for i in range(len(pred_particles)):
            # Calculate risk for this specific particle
            risk = get_proximity_risk(pred_particles[i], self.map_metadata)
            
            # Multiply by its probability (1/200 in our case)
            total_risk += sample_weights[i] * risk
        
        return total_risk * self.risk_penalty_factor

