import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, SetEnvironmentVariable, ExecuteProcess, TimerAction, LogInfo

def generate_launch_description():

    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    yaw_pose = LaunchConfiguration('yaw_pose', default='0.0')

    # Package directory
    pkg_dir = get_package_share_directory('diff_drive_robot') # Make sure this is the correct package name

    # Set GZ_SIM_RESOURCE_PATH for Gazebo Harmonic to find models, adapt in case your ´models´ are stored somewhere else
    models_path = os.path.join(pkg_dir, 'models')

    gz_resource_path = SetEnvironmentVariable(
    name='GZ_SIM_RESOURCE_PATH',
    value=models_path
    )   

    # Launch configurations
    world = LaunchConfiguration('world')
    rviz = LaunchConfiguration('rviz')

    # Path to default world 
    world_path = os.path.join(pkg_dir,'worlds', 'world.sdf')

    # Launch Arguments
    declare_world = DeclareLaunchArgument(
        name='world', default_value=world_path,
        description='Full path to the world model file to load')
    
    declare_rviz = DeclareLaunchArgument(
        name='rviz', default_value='True',
        description='Opens rviz is set to True')

    # Launch Robot State Publisher Node
    urdf_path = os.path.join(pkg_dir,'urdf','robot.urdf')
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_dir,'launch','rsp_launch.py'
            )]), launch_arguments={'use_sim_time': 'true', 'urdf': urdf_path}.items()
    )

    # Launch the gazebo server to initialize the simulation
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            )]), launch_arguments={'gz_args': ['-r -s -v1 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    # Always launch the gazebo client to visualize the simulation
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            )]), launch_arguments={'gz_args': '-g '}.items()
    )

    # Run the spawner node from the gazebo_ros package. 
    spawn_diff_bot = Node(
        package='ros_gz_sim', 
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'diff_bot',
                   '-x', x_pose, '-y', y_pose, '-z', '0.2','-Y', yaw_pose],
        output='screen'
    )

    # Launch the Gazebo-ROS bridge
    bridge_params = os.path.join(pkg_dir,'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{
            'config_file': bridge_params,
            'use_sim_time': True 
        }],
        output='screen'
    )
    
    # Launch Rviz with diff bot rviz file
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'bot.rviz')
    rviz2 = GroupAction(
        condition=IfCondition(rviz),
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': True}],
            output='screen',
        )]
    )


    map_file = os.path.join(pkg_dir, 'maps', 'my_map.yaml')  # Ensure this path is correct for the SLAM generated map (see mapping_launch.py if you need to create one)
    
    amcl_params = os.path.join(pkg_dir, 'config', 'amcl.yaml')  

    # 1. MAP SERVER
    start_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file}, {'use_sim_time': True}]
    )

    # 2. AMCL
    start_amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params, {'use_sim_time': True}] 
    )
    
    # 3. LIFECYCLE MANAGER: Necessary in ROS 2 to "activate" Nav2 nodes
    start_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server', 'amcl']}]
    )    

    ready_msg = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg="[READY] Robot simulator initialized. Map and AMCL are running.")
        ]
    )

    return LaunchDescription([
        gz_resource_path,
        declare_rviz,
        declare_world,

        # Launch the nodes
        # rviz2,
        rsp,
        gazebo_server,
        gazebo_client,
        ros_gz_bridge,
        spawn_diff_bot,
        start_map_server,
        start_amcl,
        start_lifecycle_manager,
        ready_msg,
    ])