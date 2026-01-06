import numpy as np
from geometry_msgs.msg import Pose
from sklearn.cluster import KMeans
from geometry_msgs.msg import PoseArray, Pose
import tf_transformations # Often used in ROS 2 for orientation

#This file is for specialized tools that don't make decisions but perform heavy lifting or data transformation.
#Actions, refine which ones you want
ACTION_EFFECTS = {
    'forward_short': {'linear': 0.2, 'angular': 0.0},
    'rotate_left':   {'linear': 0.0, 'angular': 0.5},
    'rotate_right':  {'linear': 0.0, 'angular': -0.5},
    'spin_360':      {'linear': 0.0, 'angular': 1.0},
    'wait':          {'linear': 0.0, 'angular': 0.0}
}

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
    # pose: [x, y, theta] numpy array
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

class ParticleClusturer:
    def __init__(self, n_clusters=3):
        """
        n_clusters: The number of representative modes (Hypotheses) 
                    to extract. Based on literature, 3-5 is optimal.
        """
        self.n_clusters = n_clusters

    def pose_array_to_numpy(self, msg: PoseArray):
        """Converts a PoseArray message to a (N, 3) numpy array [x, y, yaw]"""
        data = []
        for pose in msg.poses:
            # Extract position
            x = pose.position.x
            y = pose.position.y
            
            # Convert Quaternion to Euler Yaw
            q = pose.orientation
            _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            
            data.append([x, y, yaw])
        return np.array(data)

    def get_representative_clusters(self, msg: PoseArray):
        """
        Runs K-Means clustering and returns a list of 'Representative Poses'
        and the weight (importance) of each cluster.
        """
        if len(msg.poses) < self.n_clusters:
            return None, None

        # 1. Convert to NumPy
        points = self.pose_array_to_numpy(msg)

        # 2. Run K-Means
        # Note: In a true AIF model, we'd use a GMM for variance, 
        # but K-Means is faster for real-time raycasting loops.
        kmeans = KMeans(n_clusters=self.n_clusters, n_init=10)
        kmeans.fit(points[:, :2]) # Cluster based on X, Y position

        centers = kmeans.cluster_centers_
        labels = kmeans.labels_

        # 3. Calculate Weights (What % of particles belong to each cluster)
        weights = []
        representative_poses = []

        for i in range(self.n_clusters):
            # Calculate weight
            count = np.sum(labels == i)
            weight = count / len(labels)
            weights.append(weight)

            # Get the average yaw for this cluster
            cluster_yaws = points[labels == i, 2]
            # Use circular mean to avoid issues with -pi/pi wrap-around
            avg_yaw = np.arctan2(np.mean(np.sin(cluster_yaws)), np.mean(np.cos(cluster_yaws)))

            # Create a Pose object for the cluster center
            p = Pose()
            p.position.x = centers[i][0]
            p.position.y = centers[i][1]
            
            # Convert yaw back to quaternion
            q = tf_transformations.quaternion_from_euler(0, 0, avg_yaw)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
            
            representative_poses.append(p)

        return representative_poses, weights