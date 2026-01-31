import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# Simulation Constants
G = 1.625 # m/s^2
G0 = 9.81 # m/s^2
t_total = 1000 # s

# Starting Conditions
x_start = 0 # m
v_x_start = 1_695 # m/s
y_start = 15_240 # m
v_y_start = 0 # m/s

# Target Conditions
target_x = 480_000 # m
target_v_x = 0 # m/s
target_y = 0 # m
target_v_y = 0 # m/s

class Lander:
  def __init__ (self, S, Isp, T, m_e):
    '''
    Initialize the Lander object

    :param self: The Lander object
    :param S: Initial state of the lander (numpy array of 5 elements: x, v_x, y, v_y, m)
    :param Isp: Specific impulse of the lander (s)
    :param T: Maximum thrust of the lander (N)
    :param m_e: Empty mass of the lander (kg)
    '''
    self.S = S
    self.Isp = Isp
    self.T = T
    self.m_e = m_e
  def controller(self, S):
    m = S[4]
    T_min = self.T * 0.2
    T_max = self.T * 0.6

    # Horizontal PD Controller
    Kp_x, Kd_x = 100, 0
    # Vertical PD Controller
    Kp_y, Kd_y = 100, 0
    
    u_x = Kp_x * (target_x - S[0]) + Kd_x * (target_v_x - S[1])
    u_y = m * G + Kp_y * (target_y - S[2]) + Kd_y * (target_v_y - S[3])
    u = np.array([u_x, u_y])
    mag = np.linalg.norm(u)
    if mag > T_max:
      u = u / mag * T_max
    elif mag < T_min:
      u = u / mag * T_min
    return u
  def dynamics(self, t, S):
    x, v_x, y, v_y, m = S

    # State Space Matrices
    A = np.array([[0, 1, 0, 0],   
           [0, 0, 0, 0],
           [0, 0, 0, 1],
           [0, 0, 0, 0]])
    B = np.array([[0, 0], 
           [1/m, 0],
           [0, 0],
           [0, 1/m]])
    f = np.array([0, 0, 0, -G])

    # Landed
    if y <= 0:
      return [0, 0, 0, 0, 0]
    # Out of fuel
    if S[4] <= self.m_e:
      u = [0, 0]
    else:
      # Input
      u = self.controller(S)
    # Dynamics
    x_dot = A @ np.array([x, v_x, y, v_y]) + (B @ u).flatten() + f
    # Mass
    m_dot = - np.linalg.norm(u) / (G0 * self.Isp)
    return [*x_dot, m_dot]
  def propagate(self, S, duration):
    sol = solve_ivp(lambda t, S: self.dynamics(t, S), (0, duration), self.S, method='RK45', t_eval=np.linspace(0, duration, duration * 10))
    return sol
         
Apollo = Lander(np.array([x_start, v_x_start, y_start, v_y_start, 15200]), 311, 45000, 4280)

# --- Run Simulation ---
sol = Apollo.propagate(Apollo.S, t_total)

# --- Process Results ---
landed_idx = np.where(sol.y[2] <= 0)[0]

if landed_idx.size > 0:
  idx = landed_idx[0]
  
  # Data slices for plotting
  time_data = sol.t[:idx]
  x_data = sol.y[0][:idx]
  y_data = sol.y[2][:idx]
  vx_data = sol.y[1][:idx]
  vy_data = sol.y[3][:idx]
  fuel_data = sol.y[4][:idx] - Apollo.m_e

  # Final Stats
  landed_x = x_data[-1]
  landed_vx = vx_data[-1]
  landed_vy = vy_data[-1]
  landed_time = time_data[-1]
  fuel_left = fuel_data[-1]
  impact_speed = np.sqrt(landed_vx**2 + landed_vy**2)

  # --- 1. MISSION REPORT (Text) ---
  print("\n" + "="*40)
  print(f"{'LUNAR LANDING MISSION REPORT':^40}")
  print("="*40)
  
  # Status Check
  status = "SOFT LANDING" if impact_speed < 5 else "CRITICAL IMPACT"
  print(f"STATUS:     {status}")
  print(f"Flight Time:   {landed_time:.2f} s")
  print(f"Landing Site:  {landed_x:.2f} m (Target: {target_x} m)")
  print(f"Fuel Remaining: {max(0, fuel_left):.2f} kg")
  print("-" * 40)
  print(f"Final V_x:    {landed_vx:.2f} m/s")
  print(f"Final V_y:    {landed_vy:.2f} m/s")
  print(f"Impact Speed:  {impact_speed:.2f} m/s")
  print("="*40)

  # --- 2. THE DASHBOARD (Graphs) ---
  fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))

  # TOP PLOT: Spatial Trajectory (The "Arc")
  ax1.plot(x_data, y_data, color='purple', linewidth=3, label='Lander Path')
  ax1.fill_between([target_x - 100, target_x + 100], 0, 20, color='green', alpha=0.3, label='Target Zone')
  ax1.scatter(target_x, 0, color='red', marker='X', s=200, label='Target Center')
  ax1.axhline(0, color='black', linewidth=2) # The Lunar Surface
  
  ax1.set_title('FLIGHT PATH: Altitude vs. Horizontal Distance', fontsize=14, fontweight='bold')
  ax1.set_xlabel('Horizontal Distance (m)')
  ax1.set_ylabel('Altitude (m)')
  ax1.legend()
  ax1.grid(True, linestyle='--', alpha=0.6)

  # BOTTOM PLOT: Telemetry (Altitude & Fuel vs Time)
  ax2.set_title('TELEMETRY: Altitude & Fuel vs. Time', fontsize=14, fontweight='bold')
  
  # Altitude axis
  lns1 = ax2.plot(time_data, y_data, color='purple', linewidth=2, label='Altitude (m)')
  ax2.set_xlabel('Time (s)')
  ax2.set_ylabel('Altitude (m)', color='purple')
  ax2.tick_params(axis='y', labelcolor='purple')

  # Fuel axis
  ax3 = ax2.twinx()
  lns2 = ax3.plot(time_data, fuel_data, color='red', linestyle='--', linewidth=2, label='Fuel (kg)')
  ax3.set_ylabel('Fuel Remaining (kg)', color='red')
  ax3.tick_params(axis='y', labelcolor='red')

  # Combine legends from twin axes
  lns = lns1 + lns2
  labs = [l.get_label() for l in lns]
  ax2.legend(lns, labs, loc='upper right')
  
  ax2.grid(True, linestyle='--', alpha=0.6)

  plt.tight_layout()
  plt.show()

else:
  print("!!! MISSION ALERT: LANDER LOST IN SPACE (MAX TIME REACHED) !!!")