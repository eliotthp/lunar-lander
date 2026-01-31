import numpy as np
from scipy.integrate import solve_ivp

# Simulation Constants
G = 1.625  # m/s^2
G0 = 9.81  # m/s^2
t_total = 1500  # s
x_start = 0  # m
y_start = 1000  # m


class Lander:
    def __init__(self, S, Isp, T, m_e):
        """
        Initialize the Lander object

        :param self: The Lander object
        :param S: Initial state of the lander (numpy array of 5 elements: x, v_x, y, v_y, m)
        :param Isp: Specific impulse of the lander (s)
        :param T: Maximum thrust of the lander (N)
        :param m_e: Empty mass of the lander (kg)
        """
        self.S = S
        self.Isp = Isp
        self.T = T
        self.m_e = m_e

    def controller(self, S):
        m = S[4]

        T_min = self.T * 0.2
        T_max = self.T * 0.6
        target_x = 1000
        target_v_x = 0
        target_y = 0
        target_v_y = 0

        Kp = 0.1
        Kd = 50

        u_x = Kp * (target_x - S[0]) + Kd * (target_v_x - S[1])
        u_y = m * G + Kp * (target_y - S[2]) + Kd * (target_v_y - S[3])
        u = np.array([u_x, u_y])
        u = np.clip(u, T_min, T_max)
        return u

    def dynamics(self, t, S):
        x, v_x, y, v_y, m = S

        # State Space Matrices
        A = np.array([[0, 1, 0, 0], [0, 0, 0, 0], [0, 0, 0, 1], [0, 0, 0, 0]])
        B = np.array([[0, 0], [1 / m, 0], [0, 0], [0, 1 / m]])
        f = np.array([0, 0, 0, -G])

        # Landed
        if y <= 0:
            return [0, 0, 0, 0, 0]
        # Out of fuel
        if S[2] <= self.m_e:
            u = [0, 0]
        else:
            # Input
            u = self.controller(S)
        # Dynamics
        x_dot = A @ np.array([x, v_x, y, v_y]) + (B @ u).flatten() + f
        # Mass
        m_dot = -np.linalg.norm(u) / (G0 * self.Isp)
        return [*x_dot, m_dot]

    def propagate(self, S, duration):
        sol = solve_ivp(
            lambda t, S: self.dynamics(t, S),
            (0, duration),
            self.S,
            method="RK45",
            t_eval=np.linspace(0, duration, duration * 10),
        )
        return sol


Apollo = Lander(np.array([x_start, 0, y_start, 0, 15200]), 311, 45000, 4280)

sol = Apollo.propagate(Apollo.S, t_total)
