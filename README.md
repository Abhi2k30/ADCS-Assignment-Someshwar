# ADCS-Assignment-Someshwar
“MATLAB implementation of ADCS tasks (0–5) including rigid-body dynamics, PD+I control, RW/VSCMG steering, and singularity analysis.”

## Task 0 — Rigid‑Body Attitude Dynamics (7‑State Model)
**Goal:** Implement quaternion kinematics and rigid‑body rotational dynamics.

**Scripts:**
- `attitude_dynamics.m`  
  Implements the 7‑state rigid‑body model  
  

\[
  x = [q_1,q_2,q_3,q_4,\omega_x,\omega_y,\omega_z]
  \]


- `task0_simulation.m`  
  RK4 simulation of the rigid‑body model.

**Utilities used:**  
`quat_kinematics.m`, `quat_error.m`, `quat_multiply.m`

---

## Task 1 — Reaction Wheel Momentum Envelope
**Goal:** Compute and visualize the reaction wheel momentum envelope.

**Scripts:**
- `task1_envelope.m`  
  Computes RW envelope vertices and simulates momentum trajectory.
- `plot_momentum_envelope.m`  
  3D visualization of envelope + trajectory.

---

## Task 2 — VSCMG Geometry and Steering Matrices
**Goal:** Implement geometric relationships for VSCMGs.

**Scripts:**
- `vscmg_geometry.m`  
  Returns gimbal axes `g` and spin axes `s`.
- `vscmg_spin_axes.m`  
  Computes spin axes `h(δ)` as a function of gimbal angles.
- `vscmg_steering_matrices.m`  
  Computes steering matrices **S** and **G**.

These functions are used in Tasks 4 and 5.

---

## Task 3 — PD+I Attitude Control (Rigid Body Only)
**Goal:** Design and test a quaternion PD+I controller on the rigid‑body model.

**Scripts:**
- `main_task3.m`  
  Implements:
  - Quaternion error feedback  
  - PD+I control law  
  - RK4 integration  
  - No RW or VSCMG actuation  

**Utilities used:**  
`Controller_PD_I.m`, `quat_error.m`, `attitude_dynamics.m`


## Task 5(a) — 60° Slew About Non‑Principal Axis + >80% Momentum Envelope
**Goal:** Perform a large‑angle slew using RW + VSCMG steering, ensuring momentum reaches >80% of the RW envelope.

**Scripts:**
- `task5a_high_momentum_envelope.m`  
  Implements:
  - Non‑principal axis reference  
  - RW/VSCMG torque split  
  - SRS + null‑motion  
  - Momentum tracking  
  - α(t) tracking  
  - 15‑state dynamics  

**Dynamics used:**  
`attitude_dynamics_vscmg.m`

---

## Task 5(b) — Near‑Singular Tracking
**Goal:** Start near a VSCMG singularity and track a reference while monitoring σ_min(A).

**Scripts:**
- `task5b_near_singular_tracking.m`  
  Implements:
  - Near‑singular initial gimbal angles  
  - SRS  
  - Null‑motion  
  - σ_min(A) logging  
  - α(t) logging  
  - Attitude error tracking  

**Dynamics used:**  
`attitude_dynamics_vscmg.m`

---


# 🧰 Utility Functions
**Scripts:**
- `quat_error.m`
- `quat_multiply.m`
- `quat_kinematics.m`
- `quat_to_angle_deg.m`
- `axisangle2quat.m`
- `axis_angle_to_dcm.m`
- `dcm_to_quat.m`

These provide quaternion math, coordinate transforms, and helper functions.

---
