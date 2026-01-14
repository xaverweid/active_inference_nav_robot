import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
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
        default_value='true',
        description='Enable active_inference_control node'
    )

    enable_viz_arg = DeclareLaunchArgument(
        'enable_viz', default_value='true',
        description='Show the Matplotlib Belief Monitor'
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

    # Service call to initialize global localization (and output first particlecloud) when aic is enabled after a short delay
    init_global_localization = TimerAction(
    period=2.0,  # Short delay so AIC node is fully up
    actions=[
        ExecuteProcess(
            cmd=[
                'ros2', 'service', 'call',
                '/reinitialize_global_localization',
                'std_srvs/srv/Empty',
                '{}'
            ],
            output='screen'
        )
    ]
    )

    viz_node = Node(
        package='active_inference_loc',
        executable='belief_monitor',
        name='belief_monitor',
        condition=IfCondition(LaunchConfiguration('enable_viz'))
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        enable_aic_arg,
        enable_viz_arg,
        aic_control_node,
        init_global_localization,
        viz_node
    ])
