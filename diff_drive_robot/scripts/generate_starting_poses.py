import numpy as np
import yaml
from PIL import Image
import os
from ament_index_python.packages import get_package_share_directory

def load_map(map_yaml_path):
    """Load map from YAML file and return map data, resolution, and origin."""
    with open(map_yaml_path, 'r') as f:
        map_data = yaml.safe_load(f)
    
    # Get the directory containing the YAML file
    map_dir = os.path.dirname(map_yaml_path)
    
    # Construct full path to image
    map_img_path = os.path.join(map_dir, map_data['image'])
    
    # Load the map image
    map_img = Image.open(map_img_path).convert('L')  # Convert to grayscale
    map_array = np.array(map_img)
    
    # In occupancy grids: 255 = free, 0 = occupied, 205 = unknown
    # Convert to binary: 1 = free, 0 = occupied/unknown
    free_space = (map_array >= 250).astype(np.uint8)
    
    resolution = map_data['resolution']
    origin = map_data['origin'][:2]  # [x, y] in meters
    
    return free_space, resolution, origin

def world_to_grid(x, y, resolution, origin):
    """Convert world coordinates (meters) to grid coordinates (pixels)."""
    grid_x = int((x - origin[0]) / resolution)
    grid_y = int((y - origin[1]) / resolution)
    return grid_x, grid_y

def grid_to_world(grid_x, grid_y, resolution, origin):
    """Convert grid coordinates (pixels) to world coordinates (meters)."""
    x = grid_x * resolution + origin[0]
    y = grid_y * resolution + origin[1]
    return x, y

def is_valid_position(x, y, free_space, resolution, origin, min_clearance=0.25):
    """
    Check if a position is valid (free space with minimum clearance from walls).
    
    Args:
        x, y: World coordinates in meters
        free_space: Binary occupancy grid (1 = free, 0 = occupied)
        resolution: Map resolution in meters/pixel
        origin: Map origin [x, y] in meters
        min_clearance: Minimum distance from walls in meters (default: 0.25m)
    
    Returns:
        bool: True if position is valid
    """
    # Convert clearance to grid cells
    clearance_cells = int(np.ceil(min_clearance / resolution))
    
    # Convert world coordinates to grid
    grid_x, grid_y = world_to_grid(x, y, resolution, origin)
    
    # Check if within map bounds
    height, width = free_space.shape
    if not (0 <= grid_x < width and 0 <= grid_y < height):
        return False
    
    # Check if position itself is free
    if free_space[grid_y, grid_x] == 0:
        return False
    
    # Check clearance around the position (circular region)
    for dx in range(-clearance_cells, clearance_cells + 1):
        for dy in range(-clearance_cells, clearance_cells + 1):
            # Only check within circular radius
            if dx*dx + dy*dy <= clearance_cells*clearance_cells:
                check_x = grid_x + dx
                check_y = grid_y + dy
                
                # Check bounds
                if not (0 <= check_x < width and 0 <= check_y < height):
                    return False
                
                # Check if free
                if free_space[check_y, check_x] == 0:
                    return False
    
    return True

def generate_valid_starting_poses(map_yaml_path, num_poses=1000, min_clearance=0.25):
    """
    Generate valid random starting poses with minimum wall clearance.
    
    Args:
        map_yaml_path: Path to map YAML file
        num_poses: Number of valid poses to generate
        min_clearance: Minimum distance from walls in meters (default: 0.25m)
    
    Returns:
        List of tuples: [(x, y, theta), ...]
    """
    # Load map
    free_space, resolution, origin = load_map(map_yaml_path)
    height, width = free_space.shape
    
    valid_poses = []
    max_attempts = num_poses * 100  # Prevent infinite loop
    attempts = 0
    
    print(f"Generating {num_poses} valid poses with {min_clearance}m clearance...")
    print(f"Map size: {width}x{height} pixels, resolution: {resolution}m/pixel")
    
    while len(valid_poses) < num_poses and attempts < max_attempts:
        attempts += 1
        
        # Random grid position
        grid_x = np.random.randint(0, width)
        grid_y = np.random.randint(0, height)
        
        # Convert to world coordinates
        x, y = grid_to_world(grid_x, grid_y, resolution, origin)
        
        # Random orientation
        theta = np.random.uniform(-np.pi, np.pi)
        
        # Validate position
        if is_valid_position(x, y, free_space, resolution, origin, min_clearance):
            valid_poses.append((x, y, theta))
            
            if len(valid_poses) % 10 == 0:
                print(f"Generated {len(valid_poses)}/{num_poses} poses...")
    
    if len(valid_poses) < num_poses:
        print(f"Warning: Only found {len(valid_poses)} valid poses after {max_attempts} attempts")
    else:
        print(f"Successfully generated {num_poses} valid poses!")
    
    return valid_poses

def save_poses_to_csv(poses, csv_path):
    """Save poses to CSV file."""
    import csv
    
    # Ensure directory exists
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)
    
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['x', 'y', 'theta'])  # Header
        for x, y, theta in poses:
            writer.writerow([f'{x:.6f}', f'{y:.6f}', f'{theta:.6f}'])
    
    print(f"Saved {len(poses)} poses to {csv_path}")

# Main execution
if __name__ == '__main__':
    # Define paths
    map_yaml_path = os.path.join(
        get_package_share_directory('diff_drive_robot'),
        'maps',
        'my_map.yaml'
    )
    
    csv_path = os.path.join(
        get_package_share_directory('diff_drive_robot'),
        'config',
        'starting_poses_1000.csv'
    )
    
    # Generate poses with 0.25m clearance (15cm robot radius + 10cm safety)
    poses = generate_valid_starting_poses(
        map_yaml_path=map_yaml_path,
        num_poses=1000,
        min_clearance=0.25
    )
    
    # Save to CSV
    save_poses_to_csv(poses, csv_path)
    
    # Print sample poses
    print("\nSample poses:")
    for i, (x, y, theta) in enumerate(poses[:5]):
        print(f"Pose {i+1}: x={x:.3f}m, y={y:.3f}m, theta={theta:.3f}rad")