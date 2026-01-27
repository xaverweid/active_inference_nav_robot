import numpy as np
import time

import scipy  # Added for performance tracking
from .utils import ParticleClusturer, ACTION_EFFECTS, get_map_metadata, is_pose_in_collision, get_proximity_risk, calculate_shannon_entropy, log_likelihood
from .models import predict_motion, raycast_scan
from std_msgs.msg import Float32MultiArray

class ActiveInferenceController:
    def __init__(self, logger):
        self.logger = logger
        self.metrics_pub = None
        self.clusturer = ParticleClusturer()
        self.time_delta = 1.0  
        self.map_2d = None
        self.map_metadata = None
        self.current_particles = None
        self.current_weights = None
        self.actions_dict = ACTION_EFFECTS
        self.shannon_entropy = None
        self.lidar_sigma = 0.1  # Standard deviation for LiDAR likelihood model

        # --- TUNABLE PARAMETERS (The "Personality" of your Robot) ---
        # 1. Epistemic Weight (gamma): Curiosity. 
        # Higher = Robot explores more to reduce uncertainty.
        self.alpha_epistemic = 50.0 

        # 2. Pragmatic Weight (beta): Importance of following the "Goal/Safety".
        # Higher = Robot prioritizes safety/risk avoidance.
        # Keep beta_pragmatic * risk_penalty_factor = 1000 for having 1000 as maximum risk value.
        self.beta_pragmatic = 2.0 

        # 3. Risk Penalty: The "cost" of a single collision.
        # This scales the pragmatic value before it is weighted by beta.
        self.risk_penalty_factor = 500.0


    def set_metrics_publisher(self, pub):
        self.metrics_pub = pub
        self.logger.info("Metrics publisher set in ActiveInferenceController.")

    def set_map(self, map_msg):
        self.logger.info("Processing new map message...")
        self.map_metadata = get_map_metadata(map_msg)
        raw_data = np.array(map_msg.data, dtype=np.int8)
        self.map_2d = raw_data.reshape((self.map_metadata['height'], 
                                        self.map_metadata['width']))
        self.logger.info(f"Map 2D initialized: {self.map_2d.shape}")
        
        # Log map stats for debugging
        obstacles = (self.map_2d >= 50) | (self.map_2d == -1)
        self.logger.info(f"Map stats: Shape {self.map_2d.shape}, Obstacles: {np.sum(obstacles)}, Free: {np.sum(~obstacles)}, Unknown: {np.sum(self.map_2d == -1)}")
        self.logger.info(f"Distance map range: {np.min(self.map_metadata['distance_map']):.3f} - {np.max(self.map_metadata['distance_map']):.3f} m")

    def update_belief(self, points, dt=1.0):
        
        # --- NEW: VALIDITY FILTERING ---
        if self.map_metadata is not None:
            valid_indices = []
            for i, p in enumerate(points):
                # Check if particle is in free space (0)
                # Note: using your utility function if it checks for map bounds
                if not is_pose_in_collision(p, self.map_metadata):
                    valid_indices.append(i)
            
            if len(valid_indices) > 0:
                points = points[valid_indices]
            else:
                self.logger.warn("All particles are in collision! Keeping last known good belief.")
                return

        self.current_particles = points
        self.time_delta = dt

    def is_ready(self):
        map_ok = self.map_2d is not None
        part_ok = self.current_particles is not None
        if not map_ok: self.logger.debug("Controller NOT ready: Missing Map")
        if not part_ok: self.logger.debug("Controller NOT ready: Missing Particles")
        return map_ok and part_ok

    def decide_action(self):
        if not self.is_ready():
            return None

        start_time = time.time()
        
        # 1. SAMPLE PARTICLES (For Pragmatic/Risk)
        # Since weights are equal, we take a random uniform sample. 
        # This sample naturally represents the probability density of the cloud.
        num_total = len(self.current_particles)
        sample_size = min(num_total, 200)
        
        # Randomly select indices without replacement
        sample_indices = np.random.choice(num_total, sample_size, replace=False)
        
        # These represent the "possible bodies" of the robot
        sample_particles = self.current_particles[sample_indices]
        
        # Since they are unweighted, we treat them all as equally likely (1/sample_size)
        sample_weights = np.ones(sample_size) / sample_size
         
        # 2. CONDENSE BELIEF (For Epistemic/Info Gain)
        representative_poses, rep_weights = self.clusturer.get_representative_clusters_from_gmm(
            self.current_particles
        )
        
        self.shannon_entropy = calculate_shannon_entropy(rep_weights)
        # Log how many clusters we actually found (might be < 5 if particles are tight)
        self.logger.info(f"Thinking... (Clustered into {len(representative_poses)} hypotheses)")

        efe_scores = {}
        details = {} 

        # 2. EVALUATE EFE (G)
        for action in self.actions_dict.keys():
            action_start = time.time()
            
            # Predict trajectory for clusters (Epistemic)
            pred_clusters = np.array([predict_motion(p, action, self.actions_dict, dt=self.time_delta) for p in representative_poses])
            
            # Predict trajectory for particles (Pragmatic)
            pred_particles = np.array([predict_motion(p, action, self.actions_dict, dt=self.time_delta) for p in sample_particles])
            # Calculate components
            # Note: We pass the particle predictions to pragmatic
            raw_pragmatic = self.calculate_efe_pragmatic(pred_particles, sample_weights)
            raw_epistemic = self.calculate_efe_epistemic(pred_clusters, rep_weights)

            # Calculate components
            # Add a tiny penalty for staying still to encourage movement
            if action == 'WAIT':
                raw_epistemic += 5.0

            # WEIGHTED EFE: G = (Gamma * Ambiguity) + (Beta * Risk)
            # We minimize G. 
            
            total_efe = (self.alpha_epistemic * raw_epistemic) + \
                        (self.beta_pragmatic * raw_pragmatic)
            
            efe_scores[action] = total_efe
            details[action] = {'epistemic': raw_epistemic, 'pragmatic': raw_pragmatic}
            
            self.logger.debug(f"Action [{action}] evaluated in {time.time()-action_start:.3f}s")

        # 3. SELECT ACTION
        best_action = min(efe_scores, key=efe_scores.get)
        best_detail = details[best_action]
        

        # 4. METRICS & LOGGING
        total_time = time.time() - start_time
        
        # Critical warning if the 'thought' took longer than the robot's step
        if total_time > self.time_delta:
            self.logger.warn(f"AIC Slowdown: Thought took {total_time:.2f}s, but step is {self.time_delta}s!")

        if self.metrics_pub is not None:
            metrics_msg = Float32MultiArray()
            metrics_msg.data = [
                float(best_detail['epistemic']*self.alpha_epistemic), 
                float(best_detail['pragmatic']*self.beta_pragmatic), 
                float(efe_scores[best_action])
            ]
            self.metrics_pub.publish(metrics_msg)

        # Log the breakdown so you can see WHY it picked that action
        self.logger.info(
            f"Result: [{best_action}] | Total EFE: {efe_scores[best_action]:.2f} "
            f"(Info Gain: {-best_detail['epistemic']:.2f}, Risk: {best_detail['pragmatic']:.2f}) | "
            f"Time: {total_time:.2f}s"
        )
        
        return best_action
    
    def calculate_efe_epistemic(self, predicted_poses, rep_weights):
        """
        Expected posterior entropy
        Compute the expected posterior entropy of the particle distribution after 
        a hypothetical LiDAR measurement taken at that action.
        We return the expected ambiguity (uncertainty) after taking that action, which should be minimized

        """
        pred_scans = raycast_scan(
            poses_4d=predicted_poses,
            map_2d=self.map_2d,
            map_metadata=self.map_metadata
        )  # (K, B)

        K, B = pred_scans.shape

        # weights of different cluster means
        log_weights = np.log(rep_weights + 1e-12) 

        expected_entropy = 0.0

        # Loop over hypothetical measurements
        for j in range(K):
            z_hat_j = pred_scans[j]
            # For each particle / cluster i, compute likelihood of z_hat_j
            log_w_ij = np.zeros(K)
            for i in range(K):
                ll = log_likelihood(
                    z_hat_j,
                    pred_scans[i],
                    sigma=self.lidar_sigma
                )
                log_w_ij[i] = log_weights[i] + ll

            log_w_ij -= scipy.special.logsumexp(log_w_ij)
            w_ij = np.exp(log_w_ij)

            H_j = calculate_shannon_entropy(w_ij)
            expected_entropy += rep_weights[j] * H_j

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
        risks = []  # Temporary for debugging

        for i in range(len(pred_particles)):
            # Calculate risk for this specific particle
            risk = get_proximity_risk(pred_particles[i], self.map_metadata)
            
            # Multiply by its probability (1/200 in our case)
            total_risk += sample_weights[i] * risk
            risks.append(risk)  # Collect for logging
        
        return total_risk * self.risk_penalty_factor
        