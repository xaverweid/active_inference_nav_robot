# Active Inference Localization

This package contains the core intelligence for an autonomous robot using the Active Inference Framework (AIF). Unlike traditional navigation which follows pre-defined paths, this "Brain" node treats global localization as an inference problem—choosing actions that satisfy both epistemic value (information gain) and pragmatic value (collision avoidance)


## Work in progress
The package is still being worked on and in development


## Supported on

Supported for [Ubuntu 24.04](https://releases.ubuntu.com/noble/) & [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)  compatibility with other versions has not been checked.

## Implementation

### Inputs (Observation)

The node subscribes to the following topics to form its "Belief" about the world:

    /particle_cloud (PoseArray): The posterior probability distribution of the robot's state provided by AMCL.

    /scan (LaserScan): Real-time proximity data to evaluate collision risk (Pragmatic Value).

    /odom (Odometry): Motion feedback to internalize the results of previous actions.

### Outputs (Action)

    /cmd_vel (Twist): Discrete velocity commands sent to the robot's motors based on the optimal policy selected.

## How it Works

The AIC process follows a recurring Perception-Action loop:

Belief Summarization: The node receives thousands of particles from AMCL and uses K-Means Clustering to simplify the distribution.

   
Expected Free Energy (EFE) Calculation: For each discrete action (e.g., FORWARD_SMALL, ROTATE_LEFT), the node predicts:

        Epistemic Value: How much will this move reduce my entropy? (Information Gain)

        Pragmatic Value: How likely is this move to cause a collision or move away from my target?

        Action Selection: The action with the lowest EFE is published to /cmd_vel.

## Launch & Configuration

This node is designed to be launched after the base robot simulation is active (diff_drive_robot robot_launch.py)


### Launch the AIC Brain
ros2 launch active_inference_loc aic_launch.py

