import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch file for Active Inference Control with AMCL particle_cloud as input.
    
    This launch file is the Brain of the AI node.
    """
    
    pkg_dir = get_package_share_directory('active_inference_loc') 
        
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
    
    # Active Inference Control node 
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
        aic_control_node,
    ])
