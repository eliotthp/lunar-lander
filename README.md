# Apollo Lunar Module Descent Simulation

![Language](https://img.shields.io/badge/Python-3.8%2B-blue)
![License](https://img.shields.io/badge/License-MIT-green)
![Status](https://img.shields.io/badge/Status-Active_Development-orange)

---

## 🚀 Project Overview

This project is a **high‑fidelity Guidance, Navigation, and Control (GNC) simulation of the Apollo Lunar Module (LM) powered descent**, with emphasis on **flight‑software architecture and physical realism** rather than pure trajectory optimization.

The simulation currently focuses on the **Braking Phase (Apollo P63)**, where the LM transitions from low lunar orbit into a controlled, powered descent. The goal is not merely to “make it land,” but to model how real spacecraft software is structured and validated.

Key architectural principles:

* Explicit separation of **plant (truth model)**, **navigation**, **guidance**, **control**, and **actuation**
* Clear state and frame conventions (LVLH vs polar)
* Incremental realism layered through actuator and propulsion constraints

---

## 🛠 Engineering & Technical Highlights

### 1. Dynamics & Physics Engine (Plant)

The translational dynamics are modeled in **polar coordinates** around the Moon:

* State: `[r, dr, θ, dθ, m]`
* Central gravity: $\mu / r^2$
* Full centrifugal and Coriolis coupling terms
* Continuous propellant mass depletion:
  [
  \dot{m} = -\frac{T}{I_{sp} g_0}
  ]

This layer represents the **physical truth model** and is intentionally more detailed than the guidance and control layers.

---

### 2. Navigation (LVLH Frame)

The plant state is mapped into a **Local Vertical / Local Horizontal (LVLH)** frame used by guidance and control:

* LVLH state: `[z, dz, x, dx, m]`

  * `z = r − r_moon` → altitude above lunar surface (+up)
  * `x = r · θ` → downrange distance (+forward)
* Explicit, documented sign conventions to avoid frame‑mixing errors

This mirrors real spacecraft navigation pipelines, where estimation and control do not operate directly in inertial coordinates.

---

### 3. Guidance

Guidance generates **commanded accelerations**, not thrust:

* Cubic polynomial guidance (receding horizon)
* Boundary‑condition driven (position and velocity)
* Outputs:

  * `ddz_cmd` — vertical acceleration command
  * `ddx_cmd` — horizontal acceleration command

Guidance is **hardware‑agnostic**; feasibility is enforced downstream.

---

### 4. Control (Acceleration → Thrust Allocation)

The controller converts acceleration commands into thrust magnitude and direction:

* Reconstructs local geometry (`r = r_moon + z`)
* Computes required radial and tangential thrust components
* Outputs:

  * `T_cmd` — raw thrust request
  * `α_cmd` — commanded thrust pitch angle

This reflects real flight software, where guidance requests accelerations and control allocates actuators.

---

### 5. Propulsion & Actuator Constraints

#### Descent Propulsion System (DPS) Throttle Logic

The LM Descent Propulsion System is modeled with **Apollo‑inspired throttle behavior**:

* **100% thrust** when demand ≥ 65%
* **Continuously throttleable** between 10–65%
* **Minimum throttle floor** at 10% while the engine is on
* Automatic cutoff at dry mass

This produces realistic “ride‑the‑stop” behavior during aggressive braking.

#### Mass Depletion

* Propellant mass decreases continuously with thrust
* Engine shuts down at dry mass
* Prevents nonphysical $T/m \rightarrow \infty$ behavior

---

### 6. Attitude / Gimbal Dynamics

Rather than assuming instantaneous thrust vectoring, the simulation includes a **rate‑limited actuator model**:

* Separate actuator state: `α_actual`
* Slew‑rate limiting: $|\dot{\alpha}| \le \dot{\alpha}_{max}$
* Thrust direction passed to the plant uses the **rate‑limited** angle

This models actuator realism without polluting the translational state.

---

## 📊 Simulation Visualizations

### **Trajectory Plot:**  
![Trajectory Plot](figs/trajectory.png)  
Tracks altitude vs. downrange distance relative to the landing site.
### **Telemetry Plots:**  
![Telemetry Plots](figs/telemetry.png)  
Real-time logging of Pitch ($\alpha$), Thrust (%), Velocity Components, and Propellant Mass.

## 📂 Project Structure

```bash
├── main.py             # Simulation loop and system integration
├── simulation.py       # Plant dynamics and integration
├── navigation.py       # Polar → LVLH frame conversion
├── guidance.py         # Polynomial guidance laws
├── controller.py       # Thrust and pitch allocation
├── visualization.py    # Telemetry and plotting utilities
├── environment.py      # Physical constants and vehicle parameters
├── test.py             # Standalone tests and trajectory checks
├── README.md           # Project documentation
└── LICENSE
```

---

## 🔧 Current Limitations

* Braking phase (P63) only
* Fixed time‑to‑go (not yet state‑scheduled)
* Single‑axis attitude model (pitch only)
* No terrain or landing leg dynamics

---

## 🔜 Planned Extensions

* High Gate / Low Gate phase transitions (P64‑style)
* State‑dependent time‑to‑go scheduling
* Fuel‑optimal guidance laws
* Higher‑order integrators (RK4)
* Monte‑Carlo dispersions
* Explicit rotational dynamics
* Validation against published Apollo descent telemetry

---

## 📚 References & Data Sources

1. **NASA Manned Spacecraft Center**, *Apollo Lunar Descent and Ascent Trajectories*, Nov 1970
   [https://ntrs.nasa.gov/api/citations/19700024568/downloads/19700024568.pdf](https://ntrs.nasa.gov/api/citations/19700024568/downloads/19700024568.pdf)

2. **NASA Manned Spacecraft Center**, *Apollo Experience Report - Mission Planning for Lunar Module Descent and Ascent*, Jun 1972 [https://ntrs.nasa.gov/api/citations/19720018205/downloads/19720018205.pdf](https://ntrs.nasa.gov/api/citations/19720018205/downloads/19720018205.pdf)

2. **NASA Safety & Mission Assurance**, *Apollo 11 Mission Report (Significant Incidents)*
   [https://sma.nasa.gov/SignificantIncidents/assets/a11_missionreport.pdf](https://sma.nasa.gov/SignificantIncidents/assets/a11_missionreport.pdf)

3. **Aircraft Engine Historical Society**, *The Lunar Module Descent Engine (LMDE)*
   [https://www.enginehistory.org/Rockets/RPE09.46/RPE09.46.shtml](https://www.enginehistory.org/Rockets/RPE09.46/RPE09.46.shtml)

4. **Wikipedia Contributors**, *Apollo Lunar Module* / *Descent Propulsion System*
   [https://en.wikipedia.org/wiki/Apollo_Lunar_Module](https://en.wikipedia.org/wiki/Apollo_Lunar_Module)

---

## 👤 Author

**Eliott Hendryx‑Parker**
Aerospace Engineering Student | GNC & Flight Dynamics

---

## ⚠️ Disclaimer

This project is **educational** and not flight‑certified. Numerical values are representative and intended for learning and demonstration purposes only.
