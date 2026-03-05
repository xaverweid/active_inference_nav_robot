import numpy as np
from numba import njit


# --- 1. Motion Model ---
def predict_motion(pose_4d, action_type, action_dict, dt):
    """
    Predicts the future state of a cluster center over a fixed time step dt.
    pose_4d: [x, y, cos(deatheta), sin(theta)]
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

'''
# --- 2. Simplified Raycaster (Needs the map!) ---
# Based on distance map and its metadata (resolution, origin)
def raycast_scan(poses_4d, dist_map, map_metadata, fov_deg=180, num_beams=16):
    res = map_metadata['resolution']
    ox = map_metadata['origin_x']
    oy = map_metadata['origin_y']
    h, w = dist_map.shape
    
    fov_rad = np.radians(fov_deg)
    angles = np.linspace(-fov_rad/2, fov_rad/2, num_beams, endpoint=False)
    max_range = 8.0 # needs to fit the lidar_gpu settings! /diff_drive_robot/urdf/lidar_gpu_180_sensor.xacro
    
    all_scans = []
    for p in poses_4d:
        x, y, cth, sth = p
        theta = np.arctan2(sth, cth)
        particle_ranges = []
        
        for angle in angles:
            global_angle = theta + angle
            dx_unit = np.cos(global_angle)
            dy_unit = np.sin(global_angle)
            
            dist = 0.1 # Small start offset
            while dist < max_range:
                curr_x = x + dx_unit * dist
                curr_y = y + dy_unit * dist
                
                gx = int((curr_x - ox) / res)
                gy = int((curr_y - oy) / res)
                
                if 0 <= gx < w and 0 <= gy < h:
                    # Look up the safe jump distance from your distance map
                    safe_jump = dist_map[gy, gx]
                    
                    # If safe_jump is very small, we hit a wall
                    if safe_jump < res: 
                        particle_ranges.append(dist)
                        break
                        
                    # LEAP forward by the safe distance
                    dist += safe_jump
                else:
                    particle_ranges.append(dist)
                    break
            else:
                particle_ranges.append(max_range)
        all_scans.append(particle_ranges)
    return np.array(all_scans)
'''
@njit
def raycast_scan_numba(
    poses_4d, dist_map, res, ox, oy, 
    fov_deg=180.0,
    num_beams=16,
    max_range=8.0,
    min_range=0.15,
    stddev=0.025):
    """
    Optimized Raycaster for Active Inference.
    # needs to fit the lidar_gpu settings! /diff_drive_robot/urdf/lidar_gpu_180_sensor.xacro
    max_range: 8.0 (Budget limit) 
    min_range: 0.15 (Blind zone) 
    stddev: 0.025 (Simulates cheap sensor jitter)
    """
    h, w = dist_map.shape
    num_poses = poses_4d.shape[0]
    
    # Pre-calculate angles based on FOV
    fov_rad = np.deg2rad(fov_deg)
    angles = np.linspace(-fov_rad/2, fov_rad/2, num_beams)
    
    results = np.empty((num_poses, num_beams))

    for i in range(num_poses):
        x, y = poses_4d[i, 0], poses_4d[i, 1]
        cth, sth = poses_4d[i, 2], poses_4d[i, 3]
        theta = np.arctan2(sth, cth)
        
        for j in range(num_beams):
            global_angle = theta + angles[j]
            dx = np.cos(global_angle)
            dy = np.sin(global_angle)
            
            # Start at the edge of the 'blind zone'
            dist = min_range
            hit = False
            
            while dist < max_range:
                # Sphere tracing leap
                gx = int((x + dx * dist - ox) / res)
                gy = int((y + dy * dist - oy) / res)
                
                if 0 <= gx < w and 0 <= gy < h:
                    safe_jump = dist_map[gy, gx]
                    
                    if safe_jump < res: # We hit a wall
                        # Add Gaussian noise to the 'mental prediction' 
                        # This reflects the agent's expectation of a noisy world
                        noise = np.random.normal(0, stddev)
                        results[i, j] = dist + noise
                        hit = True
                        break
                    
                    dist += safe_jump
                else:
                    # Off map behaves like max range or wall hit
                    results[i, j] = dist
                    hit = True
                    break
            
            if not hit:
                results[i, j] = max_range
                
    return results