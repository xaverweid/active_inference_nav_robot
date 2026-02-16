import numpy as np
from sklearn.mixture import GaussianMixture
import inspect
from nav2_msgs.msg import ParticleCloud
import scipy.ndimage as ndi

print(inspect.signature(GaussianMixture.fit))

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

    def get_representative_clusters_from_gmm(self, points_4d, weights):
        """
        Runs GMM on weighted ROS particles
        Input: 
            - points_4d (N, 4) -> [x, y, cos, sin]
            - weights (N,) -> particle weights (should sum to 1.0)
        Output: 
            - cluster_poses: The center (mean) of each Gaussian
            - cluster_weights: The 'importance' of each Gaussian (0.0 to 1.0)
            - cluster_variances: The variance of [x, y] for each Gaussian (K, 2)
        """
        # Validate inputs
        if len(points_4d) != len(weights):
            raise ValueError(
                f"Length mismatch: points_4d has {len(points_4d)} elements "
                f"but weights has {len(weights)} elements. They must be equal."
            )
        
        if len(points_4d) == 0:
            raise ValueError("Cannot cluster empty points array")
        
        # Resample points according to weights (Manual weighting for scikit-learn)
        n_samples = len(points_4d)
        indices = np.random.choice(
            n_samples, 
            size=n_samples, 
            replace=True, 
            p=weights
        )
        resampled_points = points_4d[indices]
        
        # 1. Initialize GMM
        self.gmm = GaussianMixture(
            n_components=self.n_clusters, 
            covariance_type='full', 
            max_iter=100,
            random_state=42
        )
        
        # 2. Fit to the resampled points
        self.gmm.fit(resampled_points)
        
        # 3. Extract Results
        cluster_poses = self.gmm.means_  # Shape (K, 4) -> [x, y, cos, sin]
        cluster_weights = self.gmm.weights_ # Shape (K,) -> weight of each cluster
        
        # 4. Extract Variances
        all_covariances = self.gmm.covariances_
        var_x = all_covariances[:, 0, 0] # Variance of X for all clusters
        var_y = all_covariances[:, 1, 1] # Variance of Y for all clusters

        cluster_variances = np.column_stack((var_x, var_y))

        return cluster_poses, cluster_weights, cluster_variances
        
# Utility Functions (non-class): generic ROS/map/geometry conversions

def cloud_to_numpy(msg: ParticleCloud):
    """
    Converts a nav2_msgs/ParticleCloud message to: 
    - points: (N, 4) numpy array [x, y, cos(yaw), sin(yaw)]
    - weights: (N,) numpy array of normalized weights
    """
    n = len(msg.particles)
    points_4d = np.zeros((n, 4))
    weights = np.zeros(n)

    for i, p in enumerate(msg.particles):
        # Position
        points_4d[i, 0] = p.pose.position.x
        points_4d[i, 1] = p.pose.position.y
        
        # Orientation
        q = p.pose.orientation
        # Fast Quaternion to Yaw (Inline to save function call overhead)
        # yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = np.arctan2(siny_cosp, cosy_cosp)
        
        points_4d[i, 2] = np.cos(theta)
        points_4d[i, 3] = np.sin(theta)
        
        weights[i] = p.weight

    # Normalization (Vectorized)
    w_sum = np.sum(weights)
    if w_sum > 0:
        weights /= w_sum
    else:
        weights[:] = 1.0 / n

    return points_4d, weights

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
    vals = np.maximum(vals, 1e-6) # Avoid degenerate cases
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
    data = map_metadata['data']
    
    steps = int(robot_radius / res)
    grid_x = int((x - ox) / res)
    grid_y = int((y - oy) / res)
    
    for dx in range(-steps, steps + 1):
        for dy in range(-steps, steps + 1):
            curr_x = grid_x + dx
            curr_y = grid_y + dy
            
            # Check bounds
            if not (0 <= curr_x < map_metadata['width'] and 0 <= curr_y < map_metadata['height']):
                return True  # Out of bounds = collision
            
            # Check obstacle
            if data[curr_y, curr_x] >= threshold or data[curr_y, curr_x] == -1:
                return True
    
    return False  # No collision detected

def calculate_shannon_entropy(weights):
    """
    Calculates Shannon Entropy given a list or array of weights.
    Weights should sum to 1.0.
    """
    weights = np.asarray(weights)
    # Ensure no zero weights to avoid log(0)
    weights = weights[weights > 1e-12]
    
    return -np.sum(weights * np.log(weights))

def calculate_convergence(representative_poses, rep_weights, cluster_variances):
    """
    Computes the total uncertainty of the GMM clusters.
    Args:
        representative_poses: (K, 4) array of cluster centers
        rep_weights: (K,) array of weights
        cluster_variances: (K, 2) array containing [var_x, var_y] for each cluster. 
    Returns: value in meters (lower is more converged).
    """
    if len(representative_poses) == 0:
        return 100.0 # High uncertainty if no clusters
    
    # Enforce NumPy arrays
    representative_poses = np.asarray(representative_poses)
    rep_weights = np.asarray(rep_weights)
    
   # 1. Variance BETWEEN clusters
    coords = representative_poses[:, :2]
    weighted_mean = np.average(coords, axis=0, weights=rep_weights)
    sq_diff = (coords - weighted_mean)**2
    variance_between_means = np.average(sq_diff, axis=0, weights=rep_weights)
    
    # 2. Variance WITHIN clusters (The "Internal Spread")
    if cluster_variances is not None:
        cluster_variances = np.asarray(cluster_variances) # Shape (K, 2)
        variance_within_clusters = np.average(cluster_variances, axis=0, weights=rep_weights)
    else:
        variance_within_clusters = np.zeros(2) # Fallback if unknown
        
    # 3. Total Variance = Between + Within
    total_var = variance_between_means + variance_within_clusters
    
    # 4. Total Standard Deviation (Euclidean)
    total_std = np.sqrt(np.sum(total_var))
    
    return float(total_std)


