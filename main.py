import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp

# Simulation Constants
G = 1.625  # m/s^2
G0 = 9.81  # m/s^2
t_total = 1500  # s
starting_alt = 1000  # m


class Lander:
    def __init__(self, S, Isp, T, m_e):
        self.S = S
        self.Isp = Isp
        self.T = T
        self.m_e = m_e

    def controller(self, S):
        m = S[2]
        T_min = self.T * 0.2
        T_max = self.T * 0.6
        target_y = 0
        target_v = 0

        Kp = 0.1
        Kd = 50

        u = m * G + Kp * (target_y - S[0]) + Kd * (target_v - S[1])
        return np.clip(u, T_min, T_max)

    def dynamics(self, t, S):
        y, v, m = S
        # State Space Matrices
        A = np.array([[0, 1], [0, 0]])
        B = np.array([[0], [1 / m]])
        f = np.array([0, -G])
        # Landed
        if y <= 0:
            return [0, 0, 0]
        # Out of fuel
        if S[2] <= self.m_e:
            u = 0
        else:
            # Input
            u = self.controller(S)
        # Dynamics
        x_dot = A @ np.array([y, v]) + (B @ np.array([u])).flatten() + f
        # Mass
        m_dot = -u / (G0 * self.Isp)
        return [x_dot[0], x_dot[1], m_dot]

    def propagate(self, S, duration):
        sol = solve_ivp(
            lambda t, S: self.dynamics(t, S),
            (0, duration),
            self.S,
            method="RK45",
            t_eval=np.linspace(0, duration, duration * 10),
        )
        return sol


Apollo = Lander(np.array([starting_alt, 0, 15200]), 311, 45000, 4280)

sol = Apollo.propagate(Apollo.S, t_total)
# After simulation
landed_idx = np.where(sol.y[0] <= 0)[0]
if landed_idx.size > 0:
    t_landed = sol.t[landed_idx[0]]
    print(f"Touchdown confirmed at: {t_landed:.2f} s")
print(f"Total fuel consumer: {Apollo.S[2] - sol.y[2][-1]:.2f} kg")

# Plotting
fig, ax1 = plt.subplots(figsize=(10, 6))

# Left Axis: Altitude and Velocity
line1 = ax1.plot(sol.t, sol.y[0], label="Altitude (m)", color="b", linewidth=2)
line2 = ax1.plot(
    sol.t, -sol.y[1] * 60, label="Downward Velocity (m/min)", color="g", linestyle="--"
)
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Motion (m or m/min)")
ax1.set_xlim([0, t_total])
ax1.set_ylim([0, starting_alt])
ax1.grid(True, alpha=0.3)

# Right Axis: Mass
ax2 = ax1.twinx()
line3 = ax2.plot(sol.t, sol.y[2], label="Lander Mass (kg)", color="r", linewidth=2)
ax2.set_ylabel("Mass (kg)")
# Zoom the mass axis to see the depletion clearly
ax2.set_ylim([sol.y[2][-1] - 50, Apollo.S[2] + 50])

# Combining legends from both axes
lines = line1 + line2 + line3
labels = [l.get_label() for l in lines]
ax1.legend(lines, labels, loc="upper right")

plt.title("Apollo Lunar Module: 1D Powered Descent Profile")
plt.show()
