import numpy as np
import environment as env

# --- Constants & Environment ---
mu = env.mu


def get_derivatives(S, C):
    # Unpack state
    r, dr, theta, dtheta, m = S
    T, alpha = C

    # Equations of Motion
    ddr = T / m * np.cos(alpha) - mu / r**2 + r * dtheta**2
    ddtheta = 1 / r * ((T / m) * np.sin(alpha) - 2 * dr * dtheta)

    return [ddr, ddtheta]


def update_state(dt, S, dS):
    # Unpack states
    r, dr, theta, dtheta, m = S
    ddr, ddtheta = dS
    # Update velocities
    dr += ddr * dt
    dtheta += ddtheta * dt
    # Update positions
    r += dr * dt
    theta += dtheta * dt

    return [r, dr, theta, dtheta, m]


def propagate(h, dt, S, C):
    n = int(round(dt / h))
    for _ in range(n):
        dS = get_derivatives(S, C)
        S = update_state(h, S, dS)

    return S
