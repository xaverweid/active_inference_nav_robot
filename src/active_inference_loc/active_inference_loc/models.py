import numpy as np

# --- 1. Motion Model ---
def predict_motion(pose_obj, action_type):
    """
    Predicts the future state of a cluster center.
    pose_obj: A geometry_msgs/Pose object (from your Clusturer)
    action_type: String matching your ACTION_EFFECTS keys
    """
    # 1. Extract current state
    x = pose_obj.position.x
    y = pose_obj.position.y
    
    # Using your arctan2 logic for robust 2D yaw extraction from Quaternion
    theta = 2 * np.arctan2(pose_obj.orientation.z, pose_obj.orientation.w)
    
    # 2. Define step sizes (Should match your ACTION_EFFECTS values)
    # These could also be pulled directly from ACTION_EFFECTS if you import it
    linear_step = 0.15   # Meters for 'SMALL' actions
    angular_step = 0.4   # Radians for 'ROTATE' actions
    
    # 3. Apply Kinematics
    if action_type == 'FORWARD_SMALL':
        x += linear_step * np.cos(theta)
        y += linear_step * np.sin(theta)
        
    elif action_type == 'FORWARD_LARGE':
        x += (linear_step * 2) * np.cos(theta)
        y += (linear_step * 2) * np.sin(theta)
        
    elif action_type == 'ROTATE_LEFT':
        theta += angular_step
        
    elif action_type == 'ROTATE_RIGHT':
        theta -= angular_step
        
    elif action_type == 'TURN_LEFT':
        # Combined motion
        theta += angular_step * 0.5
        x += linear_step * np.cos(theta)
        y += linear_step * np.sin(theta)
        
    elif action_type == 'SPIN_360':
        # Resultant theta is theoretically the same, but we add 
        # 'Epistemic Noise' to represent odometry drift
        theta += np.random.normal(0, 0.05) 
        
    # 4. CRITICAL: Angle Normalization
    # This keeps theta between -pi and pi. 
    # Logic: tan(theta) is the same, then atan2 finds the 'clean' version.
    theta = np.arctan2(np.sin(theta), np.cos(theta))
    
    # Return as a simple numpy array for the Raycaster
    return np.array([x, y, theta])

# --- 2. Simplified Raycaster (Needs the map!) ---
# You'll need the map as a numpy array and its metadata (resolution, origin)
def raycast_scan(particles, map_msg):
    """
    Input: 
      - particles: Nx3 numpy array [[x, y, theta], ...]
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
    angles = np.linspace(0, 2*np.pi, 16, endpoint=False)
    
    # max range of lidar in meters (don't search forever)
    max_range = 10.0 
    
    all_scans = []

    # 3. Loop through every particle (Hypothesis)
    for p in particles:
        px, py, p_theta = p
        particle_ranges = []
        
        # 4. Loop through every beam for this particle
        for angle in angles:
            global_angle = p_theta + angle
            
            # Unit vector for the ray
            dx = np.cos(global_angle) * resolution 
            dy = np.sin(global_angle) * resolution 
            
            # Start position (in World Coords)
            curr_x, curr_y = px, py
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