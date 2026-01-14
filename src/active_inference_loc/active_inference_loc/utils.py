import numpy as np
from geometry_msgs.msg import Pose
from sklearn.cluster import KMeans
from matplotlib.patches import Ellipse
import tf_transformations # Often used in ROS 2 for orientation
from nav2_msgs.msg import ParticleCloud

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
    'TURN_RIGHT':    {'linear': 0.15, 'angular': -0.3}
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

    def cloud_to_numpy(self, msg: ParticleCloud):
        """
        Converts a nav2_msgs/ParticleCloud message to: 
        - points: (N, 4) numpy array [x, y, cos(yaw), sin(yaw)]
        - weights: (N,) numpy array of weights
        """
        points_4d = []
        weights = []

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
            weights.append(p.weight)

            # Normalize weights in update_belief function as needed

        return np.array(points_4d), np.array(weights)

    def get_representative_clusters_from_np(self, points_4d, weights):
        """
        Runs K-Means clustering and returns a list of 'Representative Poses'
        and the weight (importance) of each cluster.
        # K-Means on [x, y, cos_theta, sin_theta] and Normalize weights.
        """

        # K-Means on [x, y, cos_theta, sin_theta]
        kmeans = KMeans(n_clusters=self.n_clusters, n_init=10)
        labels = kmeans.fit_predict(points_4d)
        
        cluster_poses = []
        cluster_weights = []
        
        for i in range(self.n_clusters):
            # Weighted mean of particles in this cluster
            mask = (labels == i)
            if not np.any(mask): continue
            
            c_weights = weights[mask]
            c_weights /= np.sum(c_weights) # Internal normalization
            
            # Calculate weighted center
            mean_x = np.average(points_4d[mask, 0], weights=c_weights)
            mean_y = np.average(points_4d[mask, 1], weights=c_weights)
            
            # Safe angle mean
            mean_cos = np.average(points_4d[mask, 2], weights=c_weights)
            mean_sin = np.average(points_4d[mask, 3], weights=c_weights)
            
            # Conversion back to 3D yaw for the robots physical is NOT done here, but in the display, robot control, etc.
            # Return is 4D for internal consistency
            
            cluster_poses.append((mean_x, mean_y, mean_cos, mean_sin))
            cluster_weights.append(np.sum(weights[mask]))
            
        return cluster_poses, cluster_weights

# Utility Functions (non-class): generic ROS/map/geometry conversions

def ros_pose_to_np(pose_msg):
    # Convert ROS Pose to numpy array [x, y, theta]
    return np.array([pose_msg.position.x, pose_msg.position.y, 
                     2 * np.arctan2(pose_msg.orientation.z, pose_msg.orientation.w)])  # Assuming quaternion to yaw

def get_map_metadata(map_msg):
    # Extract metadata from OccupancyGrid
    resolution = map_msg.info.resolution
    origin_x = map_msg.info.origin.position.x
    origin_y = map_msg.info.origin.position.y
    width = map_msg.info.width
    height = map_msg.info.height
    data = np.array(map_msg.data).reshape((height, width))
    return {
        'resolution': resolution,
        'origin_x': origin_x,
        'origin_y': origin_y,
        'width': width,
        'height': height,
        'data': data
    }

def is_pose_in_collision(pose, map_metadata, threshold=50):
    # pose 4D: [x, y, cos(theta), sin(theta)] numpy array, but we only need x,y
    x, y = pose[0], pose[1]
    resolution = map_metadata['resolution']
    origin_x = map_metadata['origin_x']
    origin_y = map_metadata['origin_y']
    width = map_metadata['width']
    height = map_metadata['height']
    data = map_metadata['data']
    
    # Convert world coords to grid indices
    grid_x = int((x - origin_x) / resolution)
    grid_y = int((y - origin_y) / resolution)
    
    # Check bounds and occupancy
    if 0 <= grid_x < width and 0 <= grid_y < height:
        return data[grid_y, grid_x] >= threshold  # Occupied if >= threshold
    return True  # Out of bounds = collision
    
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