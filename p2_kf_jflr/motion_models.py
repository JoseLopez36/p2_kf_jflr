import numpy as np

# Motion models
# - 1: xk = [x, y, o]                  |   uk = [vx, vy, omega]
# - 2: xk = [x, y, o, vx, vy, omega]   |   uk = [vx, vy, omega]

def velocity_motion_model():
    def A(dt):
        # Define the state transition matrix A
        A = np.array([[1, 0, 0],
                      [0, 1, 0],
                      [0, 0, 1]])
        return A

    def B(mu, dt):
        # Define the control input matrix B
        B = np.array([[np.cos(mu[2]) * dt, 0, 0],
                      [np.sin(mu[2]) * dt, 0, 0],
                      [0,                  0, dt]])
        return B

    return A, B

def velocity_motion_model_2():
    def A(dt):
        # Define the state transition matrix A
        A = np.array([[1, 0, 0, dt, 0,  0],
                      [0, 1, 0, 0,  dt, 0],
                      [0, 0, 1, 0,  0,  dt],
                      [0, 0, 0, 1,  0,  0],
                      [0, 0, 0, 0,  1,  0],
                      [0, 0, 0, 0,  0,  1]])
        return A

    def B(mu, dt):
        # Define the control input matrix B
        B = np.array([[0, 0, 0],
                      [0, 0, 0],
                      [0, 0, 0],
                      [0, 0, 0],
                      [0, 0, 0],
                      [0, 0, 0]])
        return B

    return A, B
