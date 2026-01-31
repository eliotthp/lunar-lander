import numpy as np
from scipy.integrate import solve_ivp

G = 1.625  # m/s^2
G0 = 9.81  # m/s^2
dt = 0.1  # s
t_total = 100  # s


class Lander:
    def __init__(self, S, Isp, T, m_e):
        self.S = S
        self.Isp = Isp
        self.T = T
        self.m_e = m_e
        self.m_p = S[2] - m_e

    def controller(self, S):
        Kp = 1
        error = Kp * (0 - S[0])
        return error

    def dynamics(self, S):
        m = S[2]
        # State Space Matrices
        A = np.array([[0, 1], [0, 0]])
        B = np.array([[0], [1 / m]])
        f = np.array([0, -G])
        C = np.array([1, 0])
        # Input
        u = m * G
        # Dynamics
        x_dot = A @ S[0:2] + B @ u
        # Mass
        m_dot = u / (G0 * self.Isp)
        return [x_dot[0], x_dot[1], m_dot]

    def propagate(self, S, duration):
        solve = solve_ivp(self.dynamics, (0, duration), S, method="RK45")
        return solve


Apollo = Lander(np.array([10000, 0, 15200]), 311, 45000, 4280)

for t in np.arange(0, t_total, dt):
    print(f"Time: {t:.1f}s | State: {Apollo.S}")
