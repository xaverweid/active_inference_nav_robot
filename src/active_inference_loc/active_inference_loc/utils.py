import numpy as np
from geometry_msgs.msg import Pose

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