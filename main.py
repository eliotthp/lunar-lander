import numpy as np
from scipy.integrate import solve_ivp
import controller as ctrl  # Importing the controller module
import visualization as viz  # Importing the visualization module
import environment as env  # Importing the enviromental variables

# --- Constants & Environment ---
G_earth = env.G_earth
r_moon = env.r_moon
mu = env.mu
Isp = env.Isp
m_empty = env.m_empty
m0 = env.m0
T_max = env.T_max

# --- Initial Conditions (From Apollo 11 Event B) ---
r0 = r_moon + 14_878  # m
dr0 = -1.22  # m/s
theta0 = np.radians(40)  # rad
dtheta0 = -np.sqrt(mu / (r_moon + 14_878)) / (r_moon + 14_878)  # rad/s
alpha0 = np.pi / 2  # rad

# --- Target Conditions ---
zf = [2346.96, r_moon]  # m
dzf = [-44.2, 0]  # m/s
xf = [400_000, 480_000]  # m
dxf = [-44.2, 0]  # m/s
targets = np.array([zf, dzf, xf, dxf])

S0 = [r0, dr0, theta0, dtheta0, m0, alpha0]


# --- Functions ---
def dynamics(t, S):
    """
    Calculates the derivatives of the state vector for the lunar lander.

    Args:
        t (float): Current simulation time.
        S (list): Current state vector [r, dr, theta, dtheta, m, alpha].

    Returns:
        list: Derivatives of the state vector [dr, ddr, dtheta, ddtheta, dm, dalpha].
    """
    r, dr, theta, dtheta, m, alpha = S

    T_cmd, alpha_cmd = ctrl.control(t, S, targets[:, 0].tolist())

    # Fuel Guardrail
    if m <= m_empty:
        T_cmd, dm = 0, 0
    else:
        dm = -T_cmd / (G_earth * Isp)

    # Equations of Motion
    ddr = (T_cmd / m) * np.cos(alpha) - mu / r**2 + r * dtheta**2
    ddtheta = (1 / r) * ((T_cmd / m) * np.sin(alpha) - 2 * dr * dtheta)
    dalpha = alpha_cmd

    return [dr, ddr, dtheta, ddtheta, dm, dalpha]


# --- Simulation ---
def surface_contact(t, S):
    """
    Event function to detect when the lander reaches the lunar surface.

    Args:
        t (float): Current simulation time.
        S (list): Current state vector.

    Returns:
        float: Altitude above the lunar surface (zero at contact).
    """
    return S[0] - r_moon


surface_contact.terminal = True

sol = solve_ivp(
    lambda t, S: dynamics(t, S),
    (0, 1000),
    S0,
    method="RK45",
    events=[surface_contact],
    max_step=1,
)

# --- Convert for plotting ---
z = sol.y[0] - r_moon
dz = sol.y[1]
x = r_moon * sol.y[2]
dx = r_moon * sol.y[3]

m_p = sol.y[4] - m_empty

T_cmd_hist = []
alpha_cmd_hist = []

for i in range(len(sol.t)):
    T_cmd, alpha_cmd = ctrl.control(sol.t[i], sol.y[:, i], targets[:, 0].tolist())
    T_cmd_hist.append(T_cmd)
    alpha_cmd_hist.append(alpha_cmd)

# --- Plotting ---
viz.trajectory(np.rad2deg(sol.y[2]), sol.y[0] - r_moon)
viz.telemetry(
    sol.t,
    [dz, dx],
    np.rad2deg(alpha_cmd_hist),
    T_cmd_hist,
    np.rad2deg(sol.y[5]),
    T_cmd_hist,
    m_p,
)
