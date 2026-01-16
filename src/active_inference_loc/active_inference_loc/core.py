import numpy as np
import time  # Added for performance tracking
from .utils import ParticleClusturer, ACTION_EFFECTS, get_map_metadata, is_pose_in_collision
from .models import predict_motion, raycast_scan
from std_msgs.msg import Float32MultiArray

class ActiveInferenceController:
    def __init__(self, logger):
        self.logger = logger
        self.metrics_pub = None
        self.clusturer = ParticleClusturer(n_clusters=5) 
        self.time_delta = 1.0  
        self.map_2d = None
        self.map_metadata = None
        self.current_particles = None
        self.current_weights = None
        self.actions_dict = ACTION_EFFECTS

        # --- TUNABLE PARAMETERS (The "Personality" of your Robot) ---
        # 1. Epistemic Weight (gamma): Curiosity. 
        # Higher = Robot explores more to reduce uncertainty.
        self.alpha_epistemic = 10.0 

        # 2. Pragmatic Weight (beta): Importance of following the "Goal/Safety".
        # Higher = Robot prioritizes safety/risk avoidance.
        self.beta_pragmatic = 1.0 

        # 3. Risk Penalty: The "cost" of a single collision.
        # This scales the pragmatic value before it is weighted by beta.
        self.risk_penalty_factor = 300.0


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

    def update_belief(self, points, weights, dt=1.0):
        weight_sum = np.sum(weights)
        if weight_sum > 1e-9:
            normalized_weights = weights / weight_sum
        else:
            self.logger.warn("Particle weights collapsed to zero!")
            return

        self.current_particles = points
        self.current_weights = normalized_weights
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
        
        # 1. CONDENSE BELIEF
        representative_poses, rep_weights = self.clusturer.get_representative_clusters_from_np(
            self.current_particles, 
            self.current_weights
        )
        
        # Log how many clusters we actually found (might be < 5 if particles are tight)
        self.logger.info(f"Thinking... (Clustered into {len(representative_poses)} hypotheses)")

        efe_scores = {}
        details = {} 

        # 2. EVALUATE EFE (G)
        for action in self.actions_dict.keys():
            action_start = time.time()
            
            # Predict
            pred_poses = np.array([
                predict_motion(p, action, self.actions_dict, dt=self.time_delta) 
                for p in representative_poses
            ])

            # Calculate components
            raw_epistemic = self.calculate_efe_epistemic(pred_poses, rep_weights)
            # Add a tiny penalty for staying still to encourage movement
            if action == 'WAIT':
                raw_epistemic += 5.0

            raw_pragmatic = self.calculate_efe_pragmatic(pred_poses, rep_weights)

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
                float(best_detail['epistemic']), 
                float(best_detail['pragmatic']), 
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
        # Raycasting is the biggest bottleneck
        ray_start = time.time()
        pred_scans = raycast_scan(
            poses_4d=predicted_poses,
            map_2d=self.map_2d,
            map_metadata=self.map_metadata
        )
        
        # If the variance is very low, information gain is low
        mean_scan = np.average(pred_scans, axis=0, weights=rep_weights)
        variance = np.average((pred_scans - mean_scan) ** 2, axis=0, weights=rep_weights)
        information_gain = np.sum(variance)

        return -float(information_gain) 

    def calculate_efe_pragmatic(self, predicted_poses, rep_weights):
        expected_risk = 0.0
        for pose, w in zip(predicted_poses, rep_weights):
            if is_pose_in_collision(pose, self.map_metadata):
                expected_risk += w
        
        return expected_risk * self.risk_penalty_factor