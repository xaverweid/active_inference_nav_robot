import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch file for Active Inference Control with AMCL localization.
    
    This launch file:
    1. Loads a pre-built map using map_server
    2. Runs AMCL (Adaptive Monte Carlo Localization) for particle-filter-based pose estimation
    3. Publishes the particle cloud (/particle_cloud) for visualization/debugging
    4. Optionally starts the active_inference_loc node
    """
    
    package_name = 'active_inference_loc'
    
    # Map file paths (adjust to your map location)
    map_yaml_path = '/home/ubuntu24/my_map.yaml'
    
    # AMCL config file
    amcl_config_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'amcl.yaml'
    )
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    enable_aic_arg = DeclareLaunchArgument(
        'enable_aic',
        default_value='false',
        description='Enable active_inference_control node'
    )
    
    # Map Server node: loads the map and publishes /map
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_yaml_path},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )
    
    # AMCL node: particle filter localization
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            amcl_config_path,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )
    
    # Active Inference Control node (optional, conditional)
    # This node subscribes to particle cloud, odometry, and scan
    # and publishes control commands to /cmd_vel
    aic_control_node = Node(
        package='active_inference_loc',
        executable='aic_node',
        name='aic_node',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        condition=IfCondition(LaunchConfiguration('enable_aic')),
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        enable_aic_arg,
        map_server,
        amcl_node,
        aic_control_node,
    ])
