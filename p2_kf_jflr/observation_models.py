import numpy as np

# Observation models:
# - 1: zk = [x, y, o]
# - 2: zk = [x, y, o, vx, vy, omega]

def odometry_observation_model():
    # Define the observation model matrix C
    C = np.array([[1, 0, 0],
                  [0, 1, 0],
                  [0, 0, 1]])
    return C

def odometry_observation_model_2():
    # Define the observation model matrix C
    C = np.array([[1, 0, 0, 0, 0, 0],
                  [0, 1, 0, 0, 0, 0],
                  [0, 0, 1, 0, 0, 0],
                  [0, 0, 0, 1, 0, 0],
                  [0, 0, 0, 0, 1, 0],
                  [0, 0, 0, 0, 0, 1]])
    return C
