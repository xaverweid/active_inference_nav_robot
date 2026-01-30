import numpy as np
import time

import scipy  # Added for performance tracking
from .utils import ParticleClusturer, ACTION_EFFECTS, get_map_metadata, is_pose_in_collision, get_proximity_risk, calculate_shannon_entropy, log_likelihood, calculate_convergence
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
        self.runtime_counter = 0
        self.convergence_parameter = 0
        self.lidar_sigma = 2.0  # Standard deviation for LiDAR likelihood model
        self.wait_streak = 0

        # --- TUNABLE PARAMETERS (The "Personality" of your Robot) ---
        # 1. Epistemic Weight (gamma): Curiosity. 
        # Higher = Robot explores more to reduce uncertainty.
        self.alpha_epistemic = 5000.0 

        # 2. Pragmatic Weight (beta): Importance of following the "Goal/Safety".
        # Higher = Robot prioritizes safety/risk avoidance.
        # Keep beta_pragmatic * risk_penalty_factor = 100 for having 100 as maximum risk value
        self.beta_pragmatic = 100.0 

        # 3. Risk Penalty: The "cost" of a single collision.
        # This scales the pragmatic value before it is weighted by beta.
        self.risk_penalty_factor = 5.0


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
        self.logger.info(f"Map metadata: width={self.map_metadata['width']}, height={self.map_metadata['height']}, resolution={self.map_metadata['resolution']}")
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
        self.logger.info("Particles were updated. New particles used for control loop")
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

        initial_particles = np.copy(self.current_particles)
        
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
        representative_poses_gmm, rep_weights_gmm = self.clusturer.get_representative_clusters_from_gmm(
            initial_particles
        )
        
        self.shannon_entropy = calculate_shannon_entropy(rep_weights_gmm)
        # Log how many clusters we actually found (might be < 5 if particles are tight)
        self.logger.info(f"Thinking... (Clustered into {len(representative_poses_gmm)} hypotheses)")

        efe_scores = {}
        details = {} 

        initial_poses_gmm=np.copy(representative_poses_gmm)
        initial_poses_pragmatic=np.copy(sample_particles)
        initial_weights_gmm=np.copy(rep_weights_gmm)
        initial_weights_pragmatic=np.copy(sample_weights)

        # 2. EVALUATE EFE (G)
        for action in self.actions_dict.keys():
            action_start = time.time()
            
            # Predict trajectory for 5 clusters (Epistemic) and epistemic value
            pred_clusters = np.array([predict_motion(p, action, self.actions_dict, dt=self.time_delta) for p in initial_poses_gmm])
            raw_epistemic = self.calculate_efe_epistemic(pred_clusters, initial_weights_gmm)

            

            # Predict trajectory for 200 particles (Pragmatic) and pragmatic value
            pred_particles = np.array([predict_motion(p, action, self.actions_dict, dt=self.time_delta) for p in initial_poses_pragmatic])
            raw_pragmatic = self.calculate_efe_pragmatic(pred_particles, initial_weights_pragmatic)
            
            total_efe = (self.alpha_epistemic * raw_epistemic) + (self.beta_pragmatic * raw_pragmatic)
            
            efe_scores[action] = total_efe
            details[action] = {'epistemic': self.alpha_epistemic * raw_epistemic, 'pragmatic': self.beta_pragmatic * raw_pragmatic}
            
            self.logger.debug(f"Action [{action}] evaluated in {time.time()-action_start:.3f}s")
            self.logger.info(f"Action [{action}] scores on epistemic {raw_epistemic*self.alpha_epistemic}, pragmatic {raw_pragmatic*self.beta_pragmatic}")


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
                self.logger.info("WAIT was chosen more than 2 times. Second best option was chosen")
        else:
            self.wait_streak = 0

        best_detail = details[best_action]

        # 4. METRICS & LOGGING
        total_time = time.time() - start_time
        self.runtime_counter += 1
        self.convergence_parameter  = calculate_convergence(representative_poses_gmm, rep_weights_gmm)
        if self.convergence_parameter < 0.20:
            self.logger.info("!!! CONVERGENCE REACHED !!!")
            return "WAIT"

        
        # Critical warning if the 'thought' took longer than the robot's step
        if total_time > self.time_delta:
            self.logger.warn(f"AIC Slowdown: Thought took {total_time:.2f}s, but step is {self.time_delta}s!")


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
        self.logger.info(
            f"Result: [{best_action}] | Total EFE: {efe_scores[best_action]:.2f} "
            f"(Expected Entropy Change: {best_detail['epistemic']:.2f}, Risk: {best_detail['pragmatic']:.2f}) | "
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
        pred_scans = raycast_scan(predicted_poses, self.map_2d, self.map_metadata)  # (K, B)

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

        for i in range(len(pred_particles)):
            # Calculate risk for this specific particle
            risk = get_proximity_risk(pred_particles[i], self.map_metadata)
            
            # Multiply by its probability (1/200 in our case)
            total_risk += sample_weights[i] * risk
        
        return total_risk * self.risk_penalty_factor
    
        