'''
    Implementation of ZMP-based walking pattern generation.
    However, the offset tracking error of ZMP in a long distance walking pattern is observed in this method.
    For more details, refer to "Introduction to Humanoid Robotics" by Shuuji Kajita.

    Background: 
        ZMP (Zero Moment Point) is a method for making a walking robot keep balance. The goal of this implementation 
    is to deduce the trajectory of CoM with given ZMPs.
'''

import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import solve_discrete_are

class WalkPattern3DGenerator(object):
    def __init__(self, sx = 0.3, sy = 0.1, N_step = 5, T_step = 1, dT = 5e-3, zc = 0.8, q = 1, r = 1e-6, g = 9.81, N = 320):
        '''
            Parameters for generating ZMP patterns:
                sx: The step length in x-axis.
                sy: The step length in y-axis.
                N_step: Total number of steps (Taking one step by either left or right foot).
                T_step: Duration (seconds) of each step.
                dT: Sampling time (seconds).
        '''
        self.sx = sx
        self.sy = sy
        self.N_step = N_step
        self.T_step = T_step
        self.dT = dT

        '''
            Parameters for the preview control system:
                dT: Sampling time (seconds).
                zc: The height of CoM.
                q, r: Positive weights.
                g: Gravity.
                N: Number of look-ahead samples in preview controller.
        '''
        self.zc = zc
        self.q = q
        self.r = r
        self.g = g
        self.N = N


    def generate(self, x0 = 0, dx0 = 0, d2x0 = 0, y0 = 0, dy0 = 0, d2y0 = 0):
        '''
            Input:
                x0, dx0, d2x0: The state vector [distance, velocity, acceleration] on x-axis.
                y0, dy0, d2y0: The state vector [distance, velocity, acceleration] on y-axis.
            Generate the trajectory of CoM that keeps the robot walking. The steps are:
                1. Create the parameter control system.
                2. Generate the ZMPs.
                3. Compute the trajectory of CoM and verify the ZMPs.
        '''
        # Create the controller.
        A, B, C = self.create_system(self.dT, self.zc, self.g)
        K, Fs = self.create_controller(A, B, C, self.r, self.q, self.N)

        # Generate the ZMPs.
        pref = self.create_ZMP_pattern(self.sx, self.sy, self.N_step, self.T_step, self.dT)
        prefx, prefy = pref[:, 0], pref[:, 1]

        # With given ZMPs, compute the trajectory of CoM.
        Xs, Pxs = self.solve_system(A, B, C, K, Fs, pref[:, 0], self.N, x0, dx0, d2x0)
        Ys, Pys = self.solve_system(A, B, C, K, Fs, pref[:, 1], self.N, y0, dy0, d2y0)

        return Xs, Ys, Pxs, Pys, prefx, prefy
        
    
    def create_ZMP_pattern(self, sx, sy, N_step, T_step, dT):
        '''
            Generate ZMP positions with given parameters.
            The trajectories:
                X-axis:
                             |----
                        |----|
                    ----|
                Y-axis:
                         |--|
                    --|  |  |  |--
                      |--|  |--|
        '''
        # Number of samples per step.
        N_sample = int(T_step / dT) 

        # Idle in the first and last step.
        patterns = np.empty([(N_step + 2) * N_sample, 2], dtype = np.float32)

        # Idle in the first step
        patterns[:N_sample, :] = 0, 0

        # Walk
        xd, yd = 0, 0
        for s in range(1, N_step + 1):
            xd = (s - 1) * sx
            yd = (-1) ** s * sy
            patterns[s * N_sample:(s + 1) * N_sample, :] = xd, yd

        # Idle in the last step
        patterns[(N_step + 1) * N_sample:, :] = xd, 0

        return patterns


    def create_system(self, dT, zc, g):
        '''
            Output: 
                A, B, C: The matrices for the digital controller.
        '''
        A = np.array([
            [1, dT, dT**2/2],
            [0, 1, dT],
            [0, 0, 1]
        ])

        B = np.array([[dT**3/6, dT**2/2, dT]]).T

        C = np.array([[1, 0, -zc/g]])

        return A, B, C


    def create_controller(self, A, B, C, r, q, N):
        '''
            The controller is considered as a tracking control problem minimizing the performance
                J = \sum_j^\infty {Q (pref_j - p_j)**2 + R u_j**2}
            Output:
                K and Fs: For creating inputs that minimize J.
        '''
        # P: Solution of the Riccati equation.
        R = r * np.eye(1)
        Q = q * C.T @ C
        P = solve_discrete_are(A, B, Q, R)

        # Create K and Fs
        tmp = np.linalg.inv(R + B.T @ P @ B) @ B.T
        K = tmp @ P @ A
        
        Fs = []
        pre = np.copy(tmp)
        AcT = (A - B @ K).T
        for n in range(N):
            Fs.append(pre @ C.T * q)
            pre = pre @ AcT
        Fs = np.array([Fs]).flatten()

        return K, Fs


    def update_state(self, A, B, C, X, U):
        '''
            Output:
                X_next: The next state vector.
                P_k: The current ZMP position.
        '''
        X_next = A @ X + B @ U
        P_k = C @ X
        return X_next, P_k


    def solve_system(self, A, B, C, K, Fs, pref, N, x0, dx0, d2x0):
        '''
            Output:
                Xs: The state vector and ZMP in all sampling time.
                ZMPs: The prediction of ZMPs.
        '''
        # The initial state vector (all zeros by default).
        X = np.array([x0, dx0, d2x0]).T

        n_zmps = len(pref)
        pref_tmp = np.append(pref, [pref[-1]] * (N - 1))

        # Go over all samples.
        Xs, ZMPs = np.zeros(n_zmps), np.zeros(n_zmps)
        for i in range(n_zmps):
            U = -np.dot(K, X) + np.dot(Fs, pref_tmp[i:i + N])
            X, ZMP = self.update_state(A, B, C, X, U)
            Xs[i], ZMPs[i] = X[0], ZMP

        return Xs, ZMPs


if __name__ == '__main__':
    generator = WalkPattern3DGenerator()
    Xs, Ys, Pxs, Pys, zmprefx, zmprefy = generator.generate()

    plt.plot(range(len(Xs)), Xs, label = 'CoM X')
    plt.plot(range(len(Pxs)), Pxs, label = 'Pred ZMP x')
    plt.plot(range(len(zmprefx)), zmprefx, label = 'Ref ZMP x')
    plt.legend()
    plt.show()

    plt.plot(range(len(Ys)), Ys, label = 'CoM Y')
    plt.plot(range(len(Pys)), Pys, label = 'Pred ZMP y')
    plt.plot(range(len(zmprefy)), zmprefy, label = 'Ref ZMP y')
    plt.legend()
    plt.show()