import numpy as np
from .utils import ParticleClusturer, ACTION_EFFECTS, get_map_metadata, is_pose_in_collision
from .models import predict_motion, raycast_scan

class ActiveInferenceController:
    def __init__(self, logger):
        self.logger = logger
        self.clusturer = ParticleClusturer(n_clusters=5) #k-means clustering
        
        # Internal State
        self.map_data = None
        self.map_metadata = None
        self.current_particles = None
        self.actions = list(ACTION_EFFECTS.keys())

    def set_map(self, map_msg):
        """Stores the map and extracts metadata for collision checking."""
        self.map_data = map_msg
        self.map_metadata = get_map_metadata(map_msg)

    def update_belief(self, pose_array_msg):
        """Updates the internal belief (particles) from AMCL."""
        # Convert ROS message to numpy using your existing utility
        self.current_particles = np.array([
            [p.position.x, p.position.y, self._get_yaw(p.orientation)] 
            for p in pose_array_msg.poses
        ])

    def is_ready(self):
        """Checks if the controller has enough data to make a decision."""
        return self.map_data is not None and self.current_particles is not None

    def decide_action(self):
        """The main AIF loop: Evaluate G (EFE) for all actions."""
        if not self.is_ready():
            return None

        # 1. CONDENSE BELIEF: Cluster particles to make raycasting fast (Strategy A)
        # Instead of 2000 particles, we raycast from 5 representative hypotheses
        representative_poses, weights = self.clusturer.get_representative_clusters_from_np(self.current_particles)

        efe_scores = {}

        for action in self.actions:
            # 2. EVALUATE EFE (G)
            epistemic = self.calculate_efe_epistemic(representative_poses, action)
            pragmatic = self.calculate_efe_pragmatic(representative_poses, action)
            
            # Total G = Risk + Ambiguity
            efe_scores[action] = epistemic + pragmatic
            
        # 3. SELECT ACTION: Find the one that minimizes EFE
        best_action = min(efe_scores, key=efe_scores.get)
        self.logger.info(f"EFE Scores: {efe_scores}")
        return best_action

    def calculate_efe_epistemic(self, clusters, action):
        """
        Information Gain (Ambiguity Reduction).
        We want to find actions that result in ESS/Number of effective Particles
        because that means the sensor data will be most informative.
        """
        # Predict future pose for each cluster center
        pred_poses = np.array([predict_motion(p, action) for p in clusters])
        
        # Simulate raycasts from those 5 predicted positions
        # This is where your robot 'imagines' what it will see
        pred_scans = np.array([raycast_scan(p, self.map_data) for p in pred_poses])
        
        # Variance across the different cluster predictions
        # High Variance = High Ambiguity = High potential for Information Gain
        variance_per_beam = np.var(pred_scans, axis=0)
        total_information_gain = np.sum(variance_per_beam)

        # We negate it because the controller seeks to MINIMIZE efe_scores
        return -total_information_gain

    def calculate_efe_pragmatic(self, clusters, action):
        """
        Pragmatic Value (Risk/Safety).
        Penalizes actions that lead into walls.
        """
        pred_poses = np.array([predict_motion(p, action) for p in clusters])
        
        collision_count = 0
        for pose in pred_poses:
            if is_pose_in_collision(pose, self.map_metadata):
                collision_count += 1
        
        # If any major hypothesis leads to a crash, apply a massive penalty
        if collision_count > 0:
            return 1000.0 * collision_count
            
        return 0.0

    def _get_yaw(self, q):
        """Helper to get yaw from quaternion."""
        import tf_transformations
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

