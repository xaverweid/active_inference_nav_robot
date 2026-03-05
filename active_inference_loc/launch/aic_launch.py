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
    
    Responsibilities:
    - Launch AIC node (decision-making)
    - Launch belief monitor (visualization)
    - Ensure subscribers are online
    - THEN trigger /reinitialize_global_localization to publish particle cloud
    """
    pkg_dir = get_package_share_directory('active_inference_loc') 
    
    algo_mode_arg = DeclareLaunchArgument(
        'algo_mode',
        default_value='active_inf_5',
        description='Algorithm mode: active_inf_5, active_inf_500, random_walk, entropy_min, d_opt_geometry, d_opt_particle'
    )
    
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
        'enable_viz', 
        default_value='true',
        description='Show the Matplotlib Belief Monitor'
    )
    
    # AIC Control Node
    aic_control_node = Node(
        package='active_inference_loc',
        executable='aic_node',
        name='aic_node',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'algo_mode': LaunchConfiguration('algo_mode')},
        ], condition=IfCondition(LaunchConfiguration('enable_aic'))
    )

    # Belief Monitor (Visualization)
    viz_node = Node(
        package='active_inference_loc',
        executable='belief_monitor',
        name='belief_monitor',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_viz'))
    )

    # **FIXED: Wait for AIC and Belief Monitor subscribers to be online, THEN trigger global localization**
    init_global_localization = TimerAction(
        period=2.0,  # Give nodes time to start and subscribe
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

    startup_msg = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'echo',
            '/aic_metrics', '--once'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        algo_mode_arg,
        use_sim_time_arg,
        enable_aic_arg,
        enable_viz_arg,
        
        # Start nodes first
        aic_control_node,
        viz_node,
        
        # **THEN trigger particle cloud publication when subscribers are ready**
        init_global_localization
    ])
