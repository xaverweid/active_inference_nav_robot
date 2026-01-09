The active_inference_loc package implements high-level cognitive control and mapping capabilities for the differential drive robot. It is designed to facilitate Active Inference—a framework where the robot chooses actions (like exploration) to minimize uncertainty and maximize information gain about its environment.

This package provides two primary functionalities:

    
    Active Inference Control (AIC): A framework for autonomous decision-making based on the belief states provided by the robot's localization system.

Key Functionalities

    Online Asynchronous Mapping: Utilizes slam_toolbox to create high-resolution maps while the robot is in motion. This allows for the creation of .yaml and .pgm files used later by the AMCL localization system.

    Active Exploration: Designed to interface with the robot's /particle_cloud and /scan topics to drive movement based on the reduction of "expected free energy" (epistemic value).

    Visualization Suite: Includes pre-configured RViz setups specifically for mapping and belief-state monitoring.

    Modular Architecture: The launch system is designed to be "opt-in," meaning it can run alongside the base robot simulation without conflict.

🛠 Software Stack
Component	Technology	Description
Mapping Engine	Slam Toolbox	Asynchronous SLAM for real-time map generation.
Control Logic	Active Inference	Custom Python nodes implementing the AIC mathematical framework.
Communication	ROS 2 Topics	Interfaces with /scan, /odom, and /particle_cloud.
Visualization	RViz2	Custom configurations for mapping (mapping.rviz).
🚦 Usage

This package is intended to be launched alongside the diff_drive_robot base simulation.
1. Discovery/Mapping Mode

To start the robot and begin creating a new map of the environment:
Bash

# Terminal 1: Launch the robot and simulation
ros2 launch diff_drive_robot robot_launch.py

# Terminal 2: Start the mapping process
ros2 launch active_inference_loc mapping_launch.py enable_mapping:=true

2. Active Inference Control (AIC)

To launch the decision-making "brain" that will control the robot's discrete actions:
Bash

ros2 launch active_inference_loc aic_launch.py
