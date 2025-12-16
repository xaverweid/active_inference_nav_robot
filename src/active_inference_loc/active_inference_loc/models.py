import numpy as np

# --- 1. Motion Model ---
def predict_motion(pose, action_type, step_size=0.5, turn_angle=np.pi/8):
    # This is a simplified model. In a real thesis, you'd use odometry noise.
    x, y, theta = pose.x, pose.y, pose.theta
    
    if action_type == 'forward_short':
        x += step_size * np.cos(theta)
        y += step_size * np.sin(theta)
    elif action_type == 'rotate_left':
        theta += turn_angle
    elif action_type == 'rotate_right':
        theta -= turn_angle
    elif action_type == 'spin_360':
        # HOWEVER, we add noise because odometry is never perfect!
        # This is CRITICAL: Spinning accumulates error. Active Inference needs to know this.
        theta += np.random.normal(0, 0.1) # Small rotational noise
    # 'stop' or 'wait' returns the current pose
    
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

    # 2. Define Laser Angles (e.g., 8 beams: 0, 45, 90, 135...)
    # 8 beams is a good balance between speed and accuracy for active inference
    angles = np.linspace(0, 2*np.pi, 8, endpoint=False)
    
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