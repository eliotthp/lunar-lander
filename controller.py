import numpy as np
import environment as env

# --- Constants & Environment ---
G_earth = env.G_earth
r_moon = env.r_moon
mu = env.mu


def control(t, S, targets):
    # Unpack states
    z, dz, x, dx, m = S
    ddz_cmd, ddx_cmd = targets
    # Placeholder variable
    y = r_moon - z
    # Components of thrust req
    Tx = 2 * dz * dx / r_moon - ddx_cmd * y / r_moon
    Tz = mu / y**2 - y * dx**2 / r_moon**2 - ddz_cmd
    # Calculate control inputs
    alpha_cmd = np.arctan2(Tx, Tz)
    T_cmd = m * np.sqrt(Tx**2 + Tz**2)

    return T_cmd, alpha_cmd
