# Active Inference Localization Algorithm

This project provides an Active Inference framework for robot global localization, split between the **diff_drive_robot** (physical simulation) and the **active_inference_loc** (Bayesian control).

### diff_drive_robot

This ROS 2 package provides a complete simulation environment for a differential drive robot using Gazebo Harmonic. It integrates the full navigation stack for localization (AMCL), map serving, and sensor bridging, designed specifically as a foundation for advanced control algorithms like Active Inference.

The primary launch file (robot_launch.py) automates the complex setup required for a mobile robot to "know where it is" within a known environment. It handles everything from spawning the physical model in a virtual world to initializing the particle filter for localization.

### active_inference_loc

This package contains the core intelligence for an autonomous robot using the Active Inference Framework (AIF). Unlike traditional navigation which follows pre-defined paths, this "Brain" node treats global localization as an inference problem—choosing actions that satisfy both epistemic value (information gain) and pragmatic value (collision avoidance)

The active inference launch file (aic_launch.py) can be used after the diff_drive_robot (robot_launch.py) is launched, and makes the robot automatically maneouver the environment to localize itself by using the particle cloud from AMCL as input and the active inference logic as control.

## Demonstration



https://github.com/user-attachments/assets/c040b72b-469a-450b-b495-5d60c4182233




## System Requirements

System: ROS2 Jazzy, Gazebo Harmonic

## AMCL Reproducibility Patch

To ensure deterministic localization behavior during Active Inference Control
experiments, we patched `nav2_amcl` to fix the random seed used during global
localization.

**Details:**
- ROS 2 distro: Jazzy 
- navigation2 commit: e70bedbf115a89d1ebbb648154aad6877462c016
- Modified file: `nav2_amcl/src/amcl_node.cpp`
- Patch file: `patches/nav2/amcl_fixed_seed.patch`

### Applying the patch

```bash
# 1. Navigate to your workspace src
cd ~/ros2_ws/src

# 2. Clone the specific Nav2 version
git clone -b jazzy [https://github.com/ros-planning/navigation2.git](https://github.com/ros-planning/navigation2.git)
cd navigation2
git checkout e70bedbf115a89d1ebbb648154aad6877462c016

# 3. Apply the scientific control patch
git apply ../active_inference_nav_robot/patches/nav2/amcl_fixed_seed.patch

# 4. Build from the workspace root
cd ~/ros2_ws
colcon build --packages-select nav2_amcl --symlink-install

# 5. Source the workspace (Crucial!)
source install/setup.bash

