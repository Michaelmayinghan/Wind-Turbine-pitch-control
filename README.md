# Wind-Turbine-pitch-control
# Wind Turbine Blade Pitch Control System Design

> **A comprehensive engineering project exploring classical and modern control theories to optimize a wind turbine pitch system under strict actuator constraints.**

## 📌 Project Overview
This repository contains the mathematical modeling, simulation, and controller design for a wind turbine blade pitch system. The primary engineering challenge was to design a highly responsive control system (minimizing rise time) while strictly ensuring the system overshoot remains below **5%** and the control voltage never exceeds the physical actuator limit of **10326 V**.

The project explores two distinct control paradigms to solve this problem:
1. **Classical Frequency-Domain Control**: Proportional (P) and Lead-Lag Compensator design.
2. **Modern State-Space Control**: Full-state feedback with automated pole-placement optimization.

---

## 🚀 Key Technical Contributions

### 1. Mathematical Modeling & Stability Analysis
* Derived the third-order open-loop transfer function $G(s)$ from fundamental electrical (Kirchhoff's law) and mechanical (Newton's second law) equations.
* Analyzed the marginal stability of the uncompensated system due to the inherent integrator pole at the origin.

### 2. Classical Control: Lead-Lag Compensator
* Designed a Lead compensator ($\beta = 0.0723$, $\tau_D = 0.2012$) to boost the system bandwidth by **4x** (target crossover frequency $\omega_c = 18.48$ rad/s) while recovering the phase margin to **$74^\circ$**.
* Implemented a Lag compensator ($\gamma = 0.4985$, $\tau_I = 0.5411$) to provide low-frequency gain, successfully bounding the steady-state velocity error.
* **Result**: Achieved an overshoot of just **2.12%**.

### 3. Modern Control: State-Space & Pole Placement
* Transformed the system into a state-space representation tracking blade pitch angle, motor angular velocity, and armature current.
* Developed an automated MATLAB simulation loop to search for the optimal natural frequency ($\omega_n = 33.00$ rad/s) that maximizes system speed without burning out the actuator.
* **Result**: Achieved an ultra-fast rise time of **0.0674 seconds** with an overshoot of **4.36%**. The peak control voltage was clamped safely at **9815.63 V** (Limit: 10326 V).

### 4. Robustness Verification
* Evaluated the closed-loop system against a relative model error perturbation $\Delta_G(s) = (s + 5) / 20$ using frequency-domain robustness criteria to map out stability boundaries.

---

## 📁 Repository Structure

* `/src`: Contains the core MATLAB simulation script (`main_control_design.m`).
* `/reports`: Contains the detailed mathematical derivations, root locus analysis, and Nyquist/Bode plots (`Analysis_Report.pdf`).
* `/img`: Visualizations of system performance (Step responses, control signal saturation, and robustness checks).

---

## ⚙️ How to Run
1. Clone this repository.
2. Open MATLAB and run `src/main_control_design.m`.
3. The script will automatically generate the Bode plots, Root Locus diagrams, and the time-domain step responses comparing the uncompensated, Lead-Lag, and State-Space systems.
