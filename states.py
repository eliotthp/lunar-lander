from dataclasses import dataclass


@dataclass
class PolarState:
    r: float
    theta: float
    dr: float
    dtheta: float
    m: float


@dataclass
class LVLHState:
    z: float
    dz: float
    x: float
    dx: float
    m: float


@dataclass
class ControlState:
    T_cmd: float
    alpha_cmd: float
    T_ctrl: float
    alpha_ctrl: float
