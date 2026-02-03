import environment as env

# --- Constants & Environment ---
r_moon = env.r_moon
mu = env.mu


def config(S):
    # Unpack state
    r, dr, theta, dtheta, m = S
    # Project to LVLH
    z = r_moon - r
    dz = -dr
    x = r_moon * theta
    dx = -r_moon * dtheta
    # Pack new state
    LVLH = [z, dz, x, dx]
    return LVLH
