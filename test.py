import simulation as sim
import environment as env

# --- Constants & Environment ---
r_moon = env.r_moon


S0 = [
    14_878 + r_moon,
    0,
    0,
    0,
    15_240,
]


S = sim.propagate(0.01, 1, S0, [0, 0])

for k in S:
    print(f"{k:.2f}")
