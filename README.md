# Active Inference Localization Algorithm

This project provides an Active Inference framework for robot global localization, split between the **diff_drive_robot** (physical simulation) and the **active_inference_loc** (Bayesian control).

---

## Demonstration



https://github.com/user-attachments/assets/c040b72b-469a-450b-b495-5d60c4182233



### Demonstration Description

* **Left Panel: Physical Simulation (Gazebo)**
  Shows the differential drive robot (purple) navigating the environment. This represents the "ground truth" of the robot's physical state.

* **Center Panel: Robot Belief Monitor**
  Visualizes the agent's internal generative model:
  * **Background:** The static `.pgm` occupancy map.
  * **Blue Particles:** The AMCL distribution representing the belief state.
  * **GMM Clusters:** The compressed belief hypotheses used by the AIC to calculate epistemic value. Watch as these clusters move and "collapse" into a single point as the robot resolves the environment's symmetry.

* **Right Panel: Live Telemetry & Diagnostics**
  A real-time dashboard displaying the mathematical "inner workings" of the controller:
  * **Global Parameters:** Current weights for Curiosity ($\alpha$) and Caution ($\beta$).
  * **Belief Metrics:** Live calculation of Shannon Entropy, spatial dispersion and convergence (threshold $< 0.35$m).
  * **AIC Policy:** Tracking the expected pragmatic and epistemic value for the chosen action.
  * **Status:** Displays current runtime, number of particles, positional and rotational error, as well as the last chosen action. 

> **Observation:** As the video progresses, the "Active" nature of the algorithm becomes apparent: the robot intentionally chooses trajectories toward geometrically unique landmarks to minimize uncertainty, evidenced by the rapid collapse of the blue particle cloud.

--- 

## Repository Structure

The project is divided into two specialized ROS 2 packages. While each contains its own detailed `README.md`, the overall architecture is split between physical simulation and Bayesian control:

### [diff_drive_robot](./diff_drive_robot) (The "Body")
Provides a complete simulation environment using **Gazebo Harmonic**. It integrates the full navigation stack for localization (AMCL), map serving, and sensor bridging.
* **Primary Launch:** `robot_launch.py`
* **Function:** Spawns the physical model and initializes the particle filter.
* [Explore the diff_drive_robot README](./diff_drive_robot/README.md)

### [active_inference_loc](./active_inference_loc) (The "Brain")
Contains the core intelligence using the **Active Inference Framework (AIF)**. This node treats global localization as an inference problem rather than a path-following task.
* **Primary Launch:** `aic_launch.py`
* **Function:** Uses the AMCL particle cloud as a belief state and applies AIF logic for optimal action selection.
* [Explore the active_inference_loc README](./active_inference_loc/README.md)

---

## Getting Started

To reproduce the experiments, you must launch the packages in the following order:

1. **Launch the Simulation:** Use `robot_launch.py` to spawn the robot and initialize the map and AMCL.
2. **Launch the Controller:** Once the particle cloud is online, use `aic_launch.py` to start the Active Inference control loop.

---

## System Requirements & Patching

System: ROS2 Jazzy, Gazebo Harmonic

### AMCL Reproducibility Patch

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

