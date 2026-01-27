import numpy as np
from sklearn.mixture import GaussianMixture
from nav2_msgs.msg import ParticleCloud
import scipy.ndimage as ndi
import matplotlib.pyplot as plt

#This file is for specialized tools that don't make decisions but perform heavy lifting or data transformation.
#Actions, refine which ones you want

#Actions: for 180 degree lidar, rotation is valid command

# Dictionary mapping action names to Twist components
# WATCH OUT, these must match your motion model exactly!

ACTION_EFFECTS = {
    'WAIT':          {'linear': 0.0,  'angular': 0.0},
    'FORWARD_SMALL': {'linear': 0.15, 'angular': 0.0},
    'FORWARD_LARGE': {'linear': 0.30, 'angular': 0.0},
    'ROTATE_LEFT':   {'linear': 0.0,  'angular': 0.4},
    'ROTATE_RIGHT':  {'linear': 0.0,  'angular': -0.4},
    'TURN_LEFT':     {'linear': 0.15, 'angular': 0.3},
    'TURN_RIGHT':    {'linear': 0.15, 'angular': -0.3},
    'BACKWARD_SMALL':{'linear': -0.15,'angular': 0.0},
}

# Particle Clustering Utility

class ParticleClusturer:
    def __init__(self, n_clusters=5):
        """
        n_clusters: The number of representative modes (Hypotheses) 
        to extract. Based on literature, 3-5 is optimal.
        Logic: Raycast from these 3-5 centers, 
            weight them by how many particles are in that cluster, and calculate the variance.

        """
        self.n_clusters = n_clusters
        self.gmm = None  # Will hold the GaussianMixture model

    def cloud_to_numpy(self, msg: ParticleCloud):
        """
        Converts a nav2_msgs/ParticleCloud message to: 
        - points: (N, 4) numpy array [x, y, cos(yaw), sin(yaw)]
        """
        points_4d = []

        for p in msg.particles:
            # Extract position
            x = p.pose.position.x
            y = p.pose.position.y
            
            # 1. Convert Quaternion to Yaw (theta)
            q = p.pose.orientation

            theta = np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 
                           1.0 - 2.0 * (q.y**2 + q.z**2))
        
            # 2. Store as 4D vector
            points_4d.append([x, y, np.cos(theta), np.sin(theta)])

        return np.array(points_4d)

    def get_representative_clusters_from_gmm(self, points_4d):
        """
        Runs GMM on unweighted ROS particles
        Input: points_4d (N, 4) -> [x, y, cos, sin]
        Output: 
       - cluster_poses: The center (mean) of each Gaussian
       - cluster_weights: The 'importance' of each Gaussian (0.0 to 1.0)
        """

        # 1. Initialize GMM
        # n_components is your self.n_clusters (e.g., 10)
        self.gmm = GaussianMixture(
            n_components=self.n_clusters, 
            covariance_type='full', 
            max_iter=100,
            random_state=42
        )

        # 2. Fit to the points
        # Since particles are unweighted, we don't need sample_weight
        self.gmm.fit(points_4d)

        # 3. Extract Results
        # Means are the representative poses (Hypotheses)
        cluster_poses = [tuple(mean) for mean in self.gmm.means_]
        
        # Weights_ are the mixing coefficients (The "Importance" of each cluster)
        # These automatically sum to 1.0
        cluster_weights = self.gmm.weights_.tolist()

        return cluster_poses, cluster_weights
        
# Utility Functions (non-class): generic ROS/map/geometry conversions

def ros_pose_to_np(pose_msg):
    # Convert ROS Pose to numpy array [x, y, theta]
    return np.array([pose_msg.position.x, pose_msg.position.y, 
                     2 * np.arctan2(pose_msg.orientation.z, pose_msg.orientation.w)])  # Assuming quaternion to yaw

def get_map_metadata(map_msg):
    """
    Extracts metadata and reshapes map data into a 2D grid.
    Returns data as (Height, Width) to allow [y, x] indexing.
    """
    info = map_msg.info
    
    # Use int8 to save memory; ROS values are -1, 0, or 100
    data_2d = np.array(map_msg.data, dtype=np.int8).reshape((info.height, info.width))
    
    # Create binary obstacle map: True for obstacles
    obstacles = (data_2d >= 50) | (data_2d == -1)
    
    # Compute distance transform: distance to nearest obstacle in meters
    # Obstacles are 0 and free space is 1 for distance transform (has the map where each pixel is the distance to nearest obstacle)
    distance_map = ndi.distance_transform_edt(~obstacles) * info.resolution
    
    # Checked for correctness, distance map looks good, same as data2d etc
    
    return {
        'resolution': info.resolution,
        'origin_x':   info.origin.position.x,
        'origin_y':   info.origin.position.y,
        'width':      info.width,
        'height':     info.height,
        'data':       data_2d,
        'distance_map': distance_map
    }


def get_covariance_ellipse(cluster_points, n_std=2.0):
    """
    Returns a Matplotlib Ellipse patch representing the covariance of a cluster.
    """
    if len(cluster_points) < 3:  # Need at least 3 points for a covariance matrix
        return None
    
    # 1. Calculate Covariance Matrix
    cov = np.cov(cluster_points[:, :2].T)
    vals, vecs = np.linalg.eigh(cov)
    
    # 2. Calculate Angle and Width/Height
    # The order of eigenvalues is ascending, so the last is the major axis
    order = vals.argsort()[::-1]
    vals, vecs = vals[order], vecs[:, order]
    
    theta = np.degrees(np.arctan2(*vecs[:, 0][::-1]))
    width, height = 2 * n_std * np.sqrt(vals)
    
    return width, height, theta

def get_proximity_risk(pose, map_metadata, safe_dist=0.5, robot_radius=0.18, sigma=None):
    """
    Utilizes the distance_map for O(1) lookup.
    distance_map gives distance to nearest obstacle in meters.
        Returns a value in [0 safe, 1 danger]:
      - 0.0 at and beyond safe_dist
      - smoothly increases below safe_dist
      - 1.0 at robot_radius or closer
    """
    if safe_dist <= robot_radius:
        raise ValueError("safe_dist must be larger than robot_radius")
    
    if sigma is None:
        sigma = (safe_dist - robot_radius) / 3.0  # 99.7% within safe_dist

    x, y = pose[0], pose[1]
    res = map_metadata['resolution']
    ox, oy = map_metadata['origin_x'], map_metadata['origin_y']
    
    # 1. Convert World Coordinates (meters) to Grid Coordinates (pixels)
    grid_x = int((x - ox) / res)
    grid_y = int((y - oy) / res)

    # 2. Boundary Check
    if not (0 <= grid_x < map_metadata['width'] and 0 <= grid_y < map_metadata['height']):
        print("Pose out of map bounds for risk calculation.")
        return 1.0 # Out of bounds is maximum risk

    # 3. Distance Lookup (meters)
    # distance_map was calculated in get_map_metadata using distance_transform_edt
    dist_to_wall = map_metadata['distance_map'][grid_y, grid_x]

    # 4. Risk Gradient Calculation
    if dist_to_wall <= robot_radius:
        # We are hitting or inside a wall
        return 1.0
    
    if dist_to_wall >= safe_dist:
        # We are in the safe zone
        return 0.0
    
    # Smooth exponential risk (normalized)
    # Anchored so:
    #   risk(robot_radius) = 1
    #   risk(safe_dist)   ≈ 0
    risk = np.exp(-(dist_to_wall - robot_radius) / sigma)

    return float(np.clip(risk, 0.0, 1.0))


def is_pose_in_collision(pose, map_metadata, threshold=50, robot_radius=0.18):
    x, y = pose[0], pose[1]
    res = map_metadata['resolution']
    ox, oy = map_metadata['origin_x'], map_metadata['origin_y']
    data = map_metadata['data'] # Assumes this is the 2D array from core.py
    
    # Check a small box around the robot based on radius
    steps = int(robot_radius / res)
    grid_x = int((x - ox) / res)
    grid_y = int((y - oy) / res)

    for dx in range(-steps, steps + 1):
        for dy in range(-steps, steps + 1):
            curr_x = grid_x + dx
            curr_y = grid_y + dy
            
            if 0 <= curr_x < map_metadata['width'] and 0 <= curr_y < map_metadata['height']:
                # Indexing data[y, x] for 2D array
                if data[curr_y, curr_x] >= threshold or data[curr_y, curr_x] == -1:
                    return True
            else:
                return True # Out of bounds
    return False

def calculate_shannon_entropy(weights):
    """
    Calculates Shannon Entropy given a list or array of weights.
    Weights should sum to 1.0.
    """
    weights = np.array(weights)
    entropy = -np.sum(weights * np.log(weights + 1e-9))
    return entropy

def log_likelihood(scan_obs, scan_pred, sigma):
    diff = scan_obs - scan_pred
    return -0.5 * np.sum((diff / sigma) ** 2)
