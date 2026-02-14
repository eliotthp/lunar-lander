import numpy as np


class Simulation:
    def __init__(self, config, S0):
        self.cfg = config
        self.state = S0

    def step(self, control, dt):
        dstate = self._get_derivatives(self.state, control)
        self.state = self._euler(self.state, dstate, dt)
        return self.state

    def _get_derivatives(self, S, C):
        """
        Calculates the second-order derivatives of the state variables.

        Args:
            S (list): Current state vector [r, dr, theta, dtheta, m].
            C (list): Control inputs [T, alpha] (Thrust and Pitch).

        Returns:
            list: Derivatives [ddr, ddtheta].
        """
        # Unpack state
        r, dr, theta, dtheta, m = S
        T, alpha = C

        # Cut thrust if propellant is exhausted
        if m - self.cfg.m_empty <= 0:
            T = 0
        # Equations of Motion in polar coordinates
        ddr = T / m * np.cos(alpha) - self.cfg.mu / r**2 + r * dtheta**2
        ddtheta = 1 / r * ((T / m) * np.sin(alpha) - 2 * dr * dtheta)
        # Mass flow rate based on ideal rocket equation
        dm = -T / (self.cfg.Isp * self.cfg.G_earth)

        return [ddr, ddtheta, dm]

    def _euler(self, S, dS, dt):
        """
        Updates the state vector using Euler integration.

        Args:
            dt (float): Time step for the update.
            S (list): Current state vector [r, dr, theta, dtheta, m].
            dS (list): Derivatives [ddr, ddtheta].

        Returns:
            list: Updated state vector.
        """
        # Unpack states
        r, dr, theta, dtheta, m = S
        ddr, ddtheta, dm = dS
        # Update velocities
        dr += ddr * dt
        dtheta += ddtheta * dt
        # Update positions
        r += dr * dt
        theta += dtheta * dt
        # Update mass
        m += dm * dt

        return [r, dr, theta, dtheta, m]
