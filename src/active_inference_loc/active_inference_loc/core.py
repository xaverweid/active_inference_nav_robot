import numpy as np
from .utils import ParticleClusturer, ACTION_EFFECTS, get_map_metadata, is_pose_in_collision
from .models import predict_motion, raycast_scan
import transformations as tf_transformations
from std_msgs.msg import Float32MultiArray


class ActiveInferenceController:
    def __init__(self, logger):
        
        """
        Args:
            logger: ROS logger (from node)
            metrics_pub: ROS publisher for /aic_metrics (optional, can be set later)
        """
        self.logger = logger
        self.metrics_pub = None
        self.clusturer = ParticleClusturer(n_clusters=5) #k-means clustering
        self.time_delta = 1.0  # Time step for discrete actions (seconds), adjust as needed
        # Internal State
        self.map_2d = None
        self.map_metadata = None
        self.current_particles = None
        self.current_weights = None
        self.actions_dict = ACTION_EFFECTS

    def set_metrics_publisher(self, pub):
        """Sets the metrics publisher (call from aic_node after creating publisher)"""
        self.metrics_pub = pub
        self.logger.info("Metrics publisher set in ActiveInferenceController.")

    def set_map(self, map_msg):
        """Stores the map and extracts metadata for collision checking."""
        self.map_metadata = get_map_metadata(map_msg)

        # 1. Convert to NumPy once
        raw_data = np.array(map_msg.data, dtype=np.int8)
        
        # 2. Reshape to 2D (Height, Width)
        # This allows direct [y, x] indexing
        self.map_2d = raw_data.reshape((self.map_metadata['height'], 
                                        self.map_metadata['width']))
        
        # 3. Optional: Store a 'collision mask' for even more speed
        # Anything > 50 is a wall. This boolean array is faster to check.
        # self.collision_mask = (self.map_2d > 50)

    def update_belief(self, points, weights, dt=1.0):
        """Updates the internal belief (particles) from AMCL.

        points: (N, 4) [x, y, cos(yaw), sin(yaw)]
        """
        # 1. Normalize weights
        weight_sum = np.sum(weights)
        if weight_sum > 1e-9:
            normalized_weights = weights / weight_sum
        else:
            self.logger.warn("Particle weights collapsed to zero! Re-initializing.")
            return

        # 2. Store for decide_action
        self.current_particles = points
        self.current_weights = normalized_weights
        self.time_delta = dt # Time step for discrete actions (seconds), adjust as needed

    def is_ready(self):
        """Checks if the controller has enough data to make a decision."""
        return self.map_2d is not None and self.current_particles is not None

    def decide_action(self):
        """The main AIF loop: Evaluate G (EFE) for all actions.

        representative_poses: (K, D) numpy array of cluster centers. D=4 for [x, y, cos(yaw), sin(yaw)]
        weights: (K,) numpy array of cluster weights.

        Returns the best action as a string.
        """

        #uses 4D ONLY

        if not self.is_ready():
            return None

        # 1. CONDENSE BELIEF: Cluster particles to make raycasting fast (Strategy A)
        # Instead of 2000 particles, we raycast from 5 representative hypotheses
        representative_poses, rep_weights = \
            self.clusturer.get_representative_clusters_from_np(
                self.current_particles, 
                self.current_weights
                )

        self.logger.debug(f"Clustered {len(self.current_particles)} particles into {len(representative_poses)} modes")

        efe_scores = {}
        details = {} # For visualization/debugging

        for action in list(self.actions_dict.keys()):
            # 2. EVALUATE EFE (G)
            # calculates here the predicted poses which then can be directly used for calculate efe functions

            pred_poses = np.array([
                predict_motion(p, action, self.actions_dict, dt=getattr(self, 'time_delta', 1.0)) 
                for p in representative_poses
            ])

            epistemic = self.calculate_efe_epistemic(pred_poses, rep_weights)
            pragmatic = self.calculate_efe_pragmatic(pred_poses, rep_weights)

            # Total G = Risk + Ambiguity
            efe_scores[action] = epistemic + pragmatic
            details[action] = {'epistemic': epistemic, 'pragmatic': pragmatic}
            
        # 3. SELECT ACTION: Find the one that minimizes EFE
        best_action = min(efe_scores, key=efe_scores.get)
        best_detail = details[best_action]

        # 4. Publish the metrics of the SELECTED action ---
       
        if self.metrics_pub is not None:
            metrics_msg = Float32MultiArray()
            metrics_msg.data = [
                float(best_detail['epistemic']), 
                float(best_detail['pragmatic']), 
                float(efe_scores[best_action])
            ]
            self.metrics_pub.publish(metrics_msg)

        self.logger.info(f"EFE Scores: {efe_scores} → Selected: {best_action}")
        
        return best_action
    
    def calculate_efe_epistemic(self, predicted_poses: np.ndarray,  rep_weights: np.ndarray):
        """
        Input: predicted poses (K, 4) and (K,) numpy arrays
        Expected Information Gain (Ambiguity Reduction).
        
        """
        
        # 1. Simulate raycasts for each predicted pose
        # This is where your robot 'imagines' what it will see
        # pred_scans is Nx8 numpy array (N particles, 8 beams per particle)
        # Each beam gives the distance to the nearest obstacle in that direction.
        # raycast_scan receives 4D poses
        pred_scans = raycast_scan(
            poses_4d=predicted_poses,
            map_msg=self.map_2d,
            map_metadata=self.map_metadata
        )             

        # 2. Weighted Variance across the different hypotheses
        # High Variance = High Ambiguity = High potential for Information Gain

        mean_scan = np.average(pred_scans, axis=0, weights=rep_weights)
        variance = np.average(
            (pred_scans - mean_scan) ** 2,
            axis=0,
            weights=rep_weights
        )

        # 4. Sum over beams
        information_gain = np.sum(variance)

        # MINIMIZE EFE ⇒ negate epistemic value
        return -information_gain

    def calculate_efe_pragmatic(self, predicted_poses: np.ndarray,  rep_weights: np.ndarray):
        """
        Pragmatic Value (Risk/Safety).
        Penalizes actions that lead into walls.
        Input: predicted poses (K, 4) and (K,) numpy arrays
        """
        expected_risk = 0.0

        for pose, w in zip(predicted_poses, rep_weights):
            if is_pose_in_collision(pose, self.map_metadata):
                expected_risk += w
        
        risk_penalty_factor = 500.0

        return expected_risk * risk_penalty_factor

    def _get_yaw(self, q):
        """Helper to get yaw from quaternion."""
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

