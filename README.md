## Active Inference Localization Algorithm - separated between the two-wheel drive robot packge (diff_drive_robot) and the Active Inference Control package (active_inference_loc)

## diff_drive_robot

This ROS 2 package provides a complete simulation environment for a differential drive robot using Gazebo Harmonic. It integrates the full navigation stack for localization (AMCL), map serving, and sensor bridging, designed specifically as a foundation for advanced control algorithms like Active Inference.

The primary launch file (robot_launch.py) automates the complex setup required for a mobile robot to "know where it is" within a known environment. It handles everything from spawning the physical model in a virtual world to initializing the particle filter for localization.

## active_inference_loc

This package contains the core intelligence for an autonomous robot using the Active Inference Framework (AIF). Unlike traditional navigation which follows pre-defined paths, this "Brain" node treats global localization as an inference problem—choosing actions that satisfy both epistemic value (information gain) and pragmatic value (collision avoidance)

The active inference launch file (aic_launch.py) can be used after the diff_drive_robot is launched, and makes the robot automatically maneouver the environment to localize itself by using the particle cloud from AMCL as input and the active inference logic as control.

### System: ROS2 Jazzy, Gazebo Harmonic

