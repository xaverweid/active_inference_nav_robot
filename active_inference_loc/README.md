# Active Inference Localization

This package contains the core intelligence for an autonomous robot using the Active Inference Framework (AIF). Unlike traditional navigation which follows pre-defined paths, this "Brain" node treats global localization as an inference problem—choosing actions that satisfy both epistemic value (information gain) and pragmatic value (collision avoidance)


## Work in progress
The package is still being worked on and in development


## Supported on

Supported for [Ubuntu 24.04](https://releases.ubuntu.com/noble/) & [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)  compatibility with other versions has not been checked.

## Active Inference–Driven Action Selection for AMCL Localization

This project integrates Active Inference for action selection with AMCL (Adaptive Monte Carlo Localization) to improve global localization by actively reducing belief uncertainty while avoiding collisions.

The key idea is:

AMCL performs localization; Active Inference selects actions that are expected to improve localization and maintain safety.

### High-Level Overview

At each control step, the robot:

Uses AMCL to estimate its current belief over poses.

Evaluates candidate actions using Expected Free Energy (EFE).

Selects the action that best trades off information gain (epistemic value) and collision avoidance (pragmatic value).

Executes the action and updates localization using real sensor data.

### Algorithm Description
1. Belief Estimation (AMCL)

AMCL processes incoming LiDAR data (and odometry)

Outputs a set of belief hypotheses, where each particle represents a possible robot pose.

2. Expected Free Energy Calculation

For each candidate action π, the Expected Free Energy G(π) is computed as the sum of Epistemic Value (uncertainty reduction) * alpha_epistemic and Pragmatic Value (collision risk) * beta_pragmatic

alpha_epistemic and beta_pragmatic are both precision parameters, and can be modify to induce different behaviors (Curiosity and Caution)

**EPISTEMIC VALUE (Entropy)**

Measures how much the action is expected to reduce localization uncertainty.

Steps:
   1. Belief Compression: Use GMM Clustering to receive N Clusters 
   2. Motion Prediction: Simulate moving all N particles according to the candidate action
   3. Virtual Sensing: Predict what LiDAR measurements each moved particle would observe
   4. Entropy Analysis: Calculate Shannon Entropy on the predicted new particle weights 

Each Action receives an Entropy Value on their predicted new particle weights. Lower entropy (Higher entropy reduction) indicates a more informative action for localization.

**PRAGMATIC VALUE (Collision Risk)**

Measures how unsafe an action (distance to obstacle of new location) is expected to be.

Steps: 
   1. Belief Compression: Take the top 200 particles
   2. Motion Prediction: Simulate moving all 200 particles according to the candidate action
   3. Estimate collision risk using an exponential decay risk function based on predicted LiDAR distances
        - Critical zone (distance to nearest obstacle ≤ robot radius): Risk = 1 (collision imminent/certain)
        - Caution zone (robot radius < distance to nearest obstacle ≤ robot radius): Risk decays exponentially from 1 to 0 (smooth transition)
        - Safe zone (distance to nearest obstacle > robot radius): Risk = 0 (far from obstacles)

This term penalizes actions that are likely to result in unsafe future positions.

3. Action Selection (Active Inference Controller)

The Active Inference Controller (AIC) selects the action that minimizes Expected Free Energy:

π∗ =arg ⁡min ⁡π(Entropy * alpha_epistemic + Collision Risk * beta_pragmatic)

This naturally balances:

- Exploration (reduce belief uncertainty)
- Safety (avoid collisions)

4. The Robot executes the selected action.

5. Localization Update (AMCL)

AMCL receives new odometry and LiDAR measurements

Particles are moved accordingly

This is where localization actually happens. Active Inference does not update the belief — it only selects actions.

6. Repeat from step 2 until Convergence has been reached.

- Convergence: the spatial spread of the GMM clusters < 0.20


## Key Takeaways

AMCL performs probabilistic localization.

Active Inference selects actions that are expected to:

Reduce belief entropy (epistemic value)

Minimize collision risk (pragmatic value)

The system enables active global localization without modifying AMCL itself.



## Launch & Configuration

This node is designed to be launched after the base robot simulation is active (diff_drive_robot robot_launch.py)


### Launch the AIC Brain
ros2 launch active_inference_loc aic_launch.py

