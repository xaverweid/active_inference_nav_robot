import numpy as np

# --- 1. Motion Model ---
def predict_motion(pose_4d, action_type, action_dict):
    """
    Predicts the future state of a cluster center.
    Gets a 4D pose object [x, y, cos(theta), sin(theta)], action type and the whole action dictionary, 
    returns also a 4D numpy array 
    action_type: String matching your ACTION_EFFECTS keys
    """
    # 1. Extract current state
    x,y, ctheta, stheta = pose_4d
    
    # Using your arctan2 logic for robust 2D yaw extraction from Quaternion
    theta = np.arctan2(stheta, ctheta) 

    # 2. Get effects directly from your ACTION_EFFECTS dictionary
    # These could also be pulled directly from ACTION_EFFECTS if you import it
    effect = action_dict.get(action_type, {'linear': 0.0, 'angular': 0.0})
    v = effect['linear']
    w = effect['angular']
    
    # 3. Standard Unicycle Kinematics (Euler Integration)
    # We update orientation first, then position (Mid-point approximation)
    new_theta = theta + w
    
    # We use the average heading for the translation to be more accurate
    move_angle = theta + (w / 2.0)
    
    new_x = x + v * np.cos(move_angle)
    new_y = y + v * np.sin(move_angle)
    
    # 4. Return as 4D [x, y, cos, sin]
    return np.array([new_x, new_y, np.cos(new_theta), np.sin(new_theta)])

# --- 2. Simplified Raycaster (Needs the map!) ---
# You'll need the map as a numpy array and its metadata (resolution, origin)
def raycast_scan(poses_4d, map_msg):
    """
    Input: 
      - particles: gets 4D numpy array of shape (N, 4) [x, y, cos(theta), sin(theta)]
      - map_msg: The full ROS OccupancyGrid message
    
    Output:
      - scans: Nx8 numpy array (N particles, 8 beams per particle)
    Each beam gives the distance to the nearest obstacle in that direction.
    Might be increased to 16 beams for more accuracy but slower computation.
    """
    
    # 1. Unpack Map Data (Do this once outside loop if possible for speed)
    width = map_msg.info.width
    height = map_msg.info.height
    resolution = map_msg.info.resolution
    origin_x = map_msg.info.origin.position.x
    origin_y = map_msg.info.origin.position.y
    map_data = np.array(map_msg.data) # Convert tuple to numpy array for speed

    # 2. Define Laser Angles (e.g., 16 beams: for 180 degree every 10-15 degree)
    # maybe reduce to 8 beams since is a good balance between speed and accuracy for active inference
    angles = np.linspace(0, 2*np.pi, 8, endpoint=False)
    
    # max range of lidar in meters (don't search forever)
    max_range = 10.0 
    
    all_scans = []

    # 3. Loop through every particle (Hypothesis)
    for p in poses_4d:
        x, y, ctheta, stheta = p
        theta = np.arctan2(stheta, ctheta)
        particle_ranges = []
        
        # 4. Loop through every beam for this particle
        for angle in angles:
            global_angle = theta + angle
            
            # Unit vector for the ray
            dx = np.cos(global_angle) * resolution 
            dy = np.sin(global_angle) * resolution 
            
            # Start position (in World Coords)
            curr_x, curr_y = x, y
            dist_accumulated = 0.0
            found_wall = False
            
            # 5. WALK THE RAY (The "Traversal")
            while dist_accumulated < max_range:
                # Convert World (meters) -> Grid (indices)
                grid_x = int((curr_x - origin_x) / resolution)
                grid_y = int((curr_y - origin_y) / resolution)
                
                # Check Bounds
                if grid_x < 0 or grid_x >= width or grid_y < 0 or grid_y >= height:
                    # Ray went off map
                    particle_ranges.append(max_range)
                    found_wall = True
                    break
                
                # Check Map Collision
                index = grid_y * width + grid_x
                if map_data[index] > 50: # Standard threshold for "Occupied"
                    # Hit a wall!
                    particle_ranges.append(dist_accumulated)
                    found_wall = True
                    break
                
                # Step forward
                curr_x += dx
                curr_y += dy
                dist_accumulated += resolution
            
            if not found_wall:
                particle_ranges.append(max_range)
        
        all_scans.append(particle_ranges)

    return np.array(all_scans)