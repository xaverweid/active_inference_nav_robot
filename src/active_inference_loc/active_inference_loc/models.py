import numpy as np

# --- 1. Motion Model ---
def predict_motion(pose_4d, action_type, action_dict, dt: float = 1.0):
    """
    Predicts the future state of a cluster center over a fixed time step dt.
    pose_4d: [x, y, cos(theta), sin(theta)]
    action_type: string key into action_dict containing linear (m/s) and angular (rad/s)
    dt: time delta in seconds the action is applied for (discrete step)
    Returns: new 4D pose numpy array [x, y, cos(theta), sin(theta)]
    """
    # 1. Extract current state
    x,y, ctheta, stheta = pose_4d
    theta = np.arctan2(stheta, ctheta) 

    # 2. Get effects directly from your ACTION_EFFECTS dictionary
    # These could also be pulled directly from ACTION_EFFECTS if you import it
    effect = action_dict.get(action_type, {'linear': 0.0, 'angular': 0.0})
    v = effect['linear']
    w = effect['angular']
    
    # 3. Standard Unicycle Kinematics (Euler Integration)
    # We update orientation first, then position (Mid-point approximation)
    delta_theta = w  * dt
    new_theta = theta + delta_theta

    
    # We use the average heading for the translation to be more accurate
    move_distance = v * dt
    move_angle = theta + delta_theta / 2.0
    
    new_x = x + move_distance * np.cos(move_angle)
    new_y = y + move_distance * np.sin(move_angle)

    return np.array([new_x, new_y, np.cos(new_theta), np.sin(new_theta)])


# --- 2. Simplified Raycaster (Needs the map!) ---
# You'll need the map as a 2D numpy array and its metadata (resolution, origin)
def raycast_scan(poses_4d, map_2d, map_metadata):
    """
    Input: 
      - poses4d: gets 4D numpy array of shape (N, 4) [x, y, cos(theta), sin(theta)]
      - map_2d: The full ROS OccupancyGrid message as a numpy array
      - map_metadata: The metadata of the map (resolution, origin)
    Output:
      - scans: Nx8 numpy array (N particles, 8 beams per particle)
    Each beam gives the distance to the nearest obstacle in that direction.
    Might be increased to 16 beams for more accuracy but slower computation.
    """
    
    # 1. Unpack Map Data (Do this once outside loop if possible for speed)
    resolution = map_metadata['resolution']
    origin_x = map_metadata['origin_x']
    origin_y = map_metadata['origin_y']
    height, width = map_2d.shape

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
            dist = 0.0
            hit = False
            
            # 5. WALK THE RAY (The "Traversal")
            while dist < max_range:
                # Convert World (meters) -> Grid (indices)
                gx = int((curr_x - origin_x) / resolution)
                gy = int((curr_y - origin_y) / resolution)
                
                # Faster bounds and collision check using 2D array
                if 0 <= gx < width and 0 <= gy < height:
                    if map_2d[gy, gx] > 50 or map_2d[gy, gx] == -1: # [row, col] is [y, x], -1 means unknown
                        particle_ranges.append(dist)
                        hit = True
                        break
                else:
                    particle_ranges.append(max_range)
                    hit = True
                    break
                
                curr_x += dx
                curr_y += dy
                dist += resolution
            
            if not hit:
                particle_ranges.append(max_range)
        
        all_scans.append(particle_ranges)

    return np.array(all_scans)