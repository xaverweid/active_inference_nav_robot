import numpy as np
from .models import predict_motion, raycast_scan
from .utils import get_map_metadata, is_pose_in_collision, ParticleClusturer

#This file is the Brain and should contain the AIF Logic (the decision loop)

def calculate_efe_epistemic(particles, map_data, action):
    # Information gain
    # 1. Predict future state (Transition)
    pred_particles = np.array([predict_motion(p, action) for p in particles])
    
    # 2. Predict future observation (Observation)
    pred_scans = raycast_scan(pred_particles, map_data)
    
    # 3. Calculate Information Gain (Disagreement across particles)
    # quantifying uncertainty across particle hypotheses 
    # Quantify Expected Certainty: For each of your 5 discrete actions, simulate the ray-casting operation to predict which map features (occupied cells, Nocc​) would be observed at the resulting pose
    #translates a Hypothesis (Particle Position) into a Predicted Observation (Laser Distance)
    # The Goal: We want to find the action that maximizes the Variance of these predicted observations.
    ####
    # High Variance in Prediction → High Disagreement among particles.
    # High Disagreement → The real sensor reading will drastically filter the cloud.
    # Drastic Filtering → Certainty (Localization).
    
    variance_per_beam = np.var(pred_scans, axis=0)
    total_information_gain = np.sum(variance_per_beam)

    # Collision Implementation as Risk (Pragmatic Value):
    # Implementation: In your core.py, when you raycast for the "Virtual Scan,"
    # check if any ray is essentially 0 (collision). put a function over it (the smaller the higher energy)
    # The Logic: "If I move forward 1m, do my particles hit a wall in the 
    # map?" -> If yes, Risk = Infinity. The resulting Free Energy (G) becomes massive, 
    # so the robot will naturally avoid that action.
    
    # EFE is minimized, so we negate Information Gain (Maximizing Info Gain)
    return -total_information_gain

def calculate_efe_pragmatic(particles, map_data, action):
    # Collision Risk: Check predicted poses
    pred_particles = np.array([predict_motion(p, action) for p in particles])
    map_metadata = get_map_metadata(map_data)
    
    collision_count = 0
    for pose in pred_particles:
        if is_pose_in_collision(pose, map_metadata):
            collision_count += 1
    
    # Option 1: Hard constraint - if any collision, infinite risk
    if collision_count > 0:
        return float('inf')
    
    # Option 2: Probabilistic - risk proportional to collision fraction
    # total_particles = len(pred_particles)
    # collision_risk = (collision_count / total_particles) * 1000  # Scale as needed
    # return collision_risk
    
    return 0  # No collision risk


def get_best_exploratory_action(particles, map_data, actions):
    # Subsample 
    indices = np.random.choice(len(particles), 30, replace=False)
    subset = particles[indices]

    scores = {}
    for action in actions:
        scores[action] = calculate_efe_epistemic(subset, map_data, action) + calculate_efe_pragmatic(subset, map_data, action)
        
    # Select action with the lowest (most negative) EFE
    best_action = min(scores, key=scores.get)
    return best_action