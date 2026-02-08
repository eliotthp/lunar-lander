import numpy as np


class altimeter:
    def __init__(self, bias, seed):
        self.bias = bias
        self.rng = np.random.default_rng(seed)

    def measure(self, z_true):
        σ_z = (0.015 * z_true + 1.52) / 3  # 1σ noise
        n = self.rng.normal(0, σ_z)  # One noise sample
        z_meas = z_true + self.bias + n
        return z_meas
