import guidance
import numpy as np

G = 6.67408e-11  # m^3/kg/s^2
G_earth = 9.81  # m/s^2
r_moon = 1737e3  # m
m_moon = 7.34767309e22  # kg
mu = G * m_moon  # m^3/s^2


def control(t, S):
    # Unpack State
    r, dr, theta, dtheta, m, alpha = S
    g = mu / r**2

    # Convert to LVLH
    z = r - r_moon
    dz = dr
    x = r_moon * theta
    dx = r_moon * dtheta

    # Braking Phase
    if z > 2346.96:
        tf = 500

        # Guidance
        _, _, ddz_cmd = guidance.poly_guidance(0, [z, 2346.96, dz, -44.20], tf)
        _, _, ddx_cmd = guidance.poly_guidance(0, [x, 400_000, dx, 44.20], tf)
    else:
        ddz_cmd = 0
        ddx_cmd = 0

    T_cmd = m * np.sqrt((ddz_cmd + g) ** 2 + ddx_cmd**2)
    alpha_cmd = np.arctan2(ddx_cmd, ddz_cmd + g)

    return T_cmd, alpha_cmd
