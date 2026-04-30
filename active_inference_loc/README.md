# Active Inference Localization

This package contains the core intelligence for an autonomous robot using the Active Inference Framework (AIF). Unlike traditional navigation which follows pre-defined paths, this "Brain" node treats global localization as an inference problem—choosing actions that satisfy both epistemic value (information gain) and pragmatic value (collision avoidance)

## Active Inference–Driven Action Selection for AMCL Localization

This project integrates Active Inference for action selection with AMCL (Adaptive Monte Carlo Localization) to improve global localization by actively reducing belief uncertainty while avoiding collisions.

AMCL performs localization; Active Inference selects actions that are expected to improve localization and maintain safety.

### Work in progress
The package is still being worked on and in development

### Supported on

Supported for [Ubuntu 24.04](https://releases.ubuntu.com/noble/) & [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)  compatibility with other versions has not been checked.

### Launch & Configuration

This node is designed to be launched after the base robot simulation is active (diff_drive_robot robot_launch.py)

### Launch the AIC Brain
ros2 launch active_inference_loc aic_launch.py

# High-Level Overview

At each control step, the robot:

Uses AMCL to estimate its current belief over poses.

Evaluates candidate actions using Expected Free Energy (EFE).

Selects the action that best trades off information gain (epistemic value) and collision avoidance (pragmatic value).

Executes the action and updates localization using real sensor data.

# Active Inference Main Algorithm (active_inf_5)

1. Belief Estimation (AMCL)

The system maintains a non-parametric belief state using a standard AMCL particle filter.

    Input: Real-world LiDAR scans, Odometry, and Map data.

    Output: A distribution of 500 to 5000 pose hypotheses (particles).

2. Expected Free Energy (EFE) Calculation

The Active Inference Controller (AIC) evaluates a set of candidate actions π (e.g., FORWARD_LARGE, TURN_LEFT) by calculating their Expected Free Energy, G(π):
G(π)≈α⋅Epistemic Value+β⋅Pragmatic Value
A. Epistemic Value (Expected Information Gain)

Goal: Select actions that maximize the distinctiveness of future sensory observations to resolve spatial ambiguity.

    Belief Compression: The particles are clustered into 5 Gaussian Mixture Model (GMM) centers to ensure computational tractability.

    Forward Simulation: For each GMM center, the algorithm predicts the future pose and performs virtual raycasting to generate a synthetic LiDAR scan.

    Likelihood Mapping: We compute a pairwise squared-difference matrix between these predicted scans. Under an isotropic Gaussian sensor model (σplanning​=0.5), this is converted into a log-likelihood matrix.

    Bayesian Posterior Update: Initial particle weights are combined with these log-likelihoods (using LogSumExp for stability) to derive a normalized posterior probability matrix.

    Entropy Analysis: We calculate the Shannon Entropy for each hypothesized state.

        Logic: If an action leads to a location where different hypotheses predict very different scans, entropy will be low upon observation, indicating high information gain.

    Final Weighting: The result is weighted by the prior probabilities of the GMM clusters to produce the final epistemic value.

B. Pragmatic Value (Expected Risk)

Goal: Ensure robot safety by evaluating collision probability across the entire hypothesis space.

    Full Distribution Evaluation: Unlike the epistemic term, risk is calculated across the top 500 particles. This prevents "averaging out" dangerous obstacles that a sparse GMM might miss.

    Path-Based Assessment: For every particle, the algorithm assesses the entire path from the current pose to the predicted pose.

    Spatial Risk Penalty: Risk is assessed using an exponential decay function based on the distance d(x,y) to the nearest obstacle:

        Collision Boundary: r=0.18m (Robot radius). Risk = 1.0.

        Danger Zone: Between r and dsafe​=0.50m. Risk follows an exponential decay controlled by σrisk​≈0.107m.

        Safe Zone: >0.50m. Risk = 0.0.

    Expected Risk: The total pragmatic value is the weighted sum of risks across the 500-particle ensemble, scaled by a global penalty factor λ to match the numerical magnitude of the epistemic term.

3. Action Selection (AIC)

The AIC selects the action that minimizes G(π):
π∗=argπmin​(α⋅Epistemic+β⋅Pragmatic)

This allows the robot to "curiously" explore ambiguous areas while "cautiously" avoiding obstacles that exist in any of its current location hypotheses.
4. Localization Update (AMCL)

The selected action π∗ is executed. AMCL receives the new physical motion data and LiDAR observations to perform its standard resampling update. The Active Inference loop effectively provides the optimal trajectory for the particle filter to converge as quickly as possible.
5. Convergence

The process repeats until the spatial spread of the GMM clusters < 0.35m, signaling that the "Kidnapped Robot" has successfully localized itself.


#  Active Inference Variants

active_inf_5 (Standard AIF): The primary implementation described in the algorithm section. It utilizes 5 GMM cluster means for epistemic calculations to balance computational tractability with localization performance.

active_inf_500 (High-Resolution AIF): A high-fidelity variant that computes the epistemic value across the top 500-particle ensemble instead of compressed GMM means. This serves as a benchmark for the information loss associated with GMM clustering.

active_inf_5_h3 (Deep AIF): Incorporates a temporal horizon of H=3. Rather than evaluating immediate next-steps, this configuration calculates the cumulative Expected Free Energy three steps into the future, allowing for more complex, multi-step exploratory trajectories.

entropy_min (Risk-Agnostic): A "purely epistemic" configuration where the pragmatic weight β is set to zero. The robot selects actions solely to minimize Shannon Entropy, disregarding collision risks to isolate the efficacy of the epistemic drive.

# Baselines & Controls

random_walk (Constrained Stochastic): A stochastic control baseline where actions are selected randomly. However, it maintains a hard collision constraint, preventing the execution of any action that would result in a direct collision.

random_walk_no_collision_avoidance (Unconstrained Stochastic): A "blind" random walk that executes actions without any regard for environmental obstacles. This serves as the absolute floor for performance and safety comparisons.

d_opt_particle (D-Optimality / Active Sensing): A frequentist baseline that selects actions to maximize Fisher Information. By assessing the sensitivity of the LiDAR data to infinitesimal changes in the robot’s pose, this configuration moves the robot toward "geometrically salient" areas (e.g., corners, doorways) based on classical active sensing principles rather than Bayesian free energy.
