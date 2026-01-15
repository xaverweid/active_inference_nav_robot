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
def raycast_scan(poses_4d, map_2d, map_metadata, fov_deg=180, num_beams=8):
    """
    fov_deg: Set this to match your real LiDAR (e.g., 180 or 360)
    num_beams: Number of rays to simulate per particle, can be 8 or 16 preferably
    """
    res = map_metadata['resolution']
    ox = map_metadata['origin_x']
    oy = map_metadata['origin_y']
    h, w = map_2d.shape

    # Adjust angles based on FOV (centered at 0)
    fov_rad = np.radians(fov_deg)
    angles = np.linspace(-fov_rad/2, fov_rad/2, num_beams, endpoint=False)
    
    max_range = 10.0 
    # Start ray 2 pixels away to avoid self-collision with the current cell
    start_offset = res * 2.0 
    
    all_scans = []

    for p in poses_4d:
        x, y, cth, sth = p
        theta = np.arctan2(sth, cth)
        particle_ranges = []
        
        for angle in angles:
            global_angle = theta + angle
            dx_unit = np.cos(global_angle)
            dy_unit = np.sin(global_angle)
            
            # Start slightly ahead of the center
            curr_x = x + dx_unit * start_offset
            curr_y = y + dy_unit * start_offset
            
            dist = start_offset
            hit = False
            
            while dist < max_range:
                gx = int((curr_x - ox) / res)
                gy = int((curr_y - oy) / res)
                
                if 0 <= gx < w and 0 <= gy < h:
                    val = map_2d[gy, gx]
                    if val > 50 or val == -1:
                        particle_ranges.append(dist)
                        hit = True
                        break
                else:
                    # Off map
                    particle_ranges.append(dist)
                    hit = True
                    break
                
                # Step by resolution
                curr_x += dx_unit * res
                curr_y += dy_unit * res
                dist += res
            
            if not hit:
                particle_ranges.append(max_range)
        
        all_scans.append(particle_ranges)

    return np.array(all_scans)