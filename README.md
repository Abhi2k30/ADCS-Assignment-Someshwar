# ADCS-Assignment-Someshwar
“MATLAB implementation of ADCS tasks (0–5) including rigid-body dynamics, PD+I control, RW/VSCMG steering, and singularity analysis.”

Task‑by‑Task Script Mapping
Task 0 — Rigid‑Body Attitude Dynamics
Scripts:

attitude_dynamics.m  
Implements the 7‑state rigid‑body model

𝑥
=
[
𝑞
1
,
𝑞
2
,
𝑞
3
,
𝑞
4
,
𝜔
𝑥
,
𝜔
𝑦
,
𝜔
𝑧
]
task0_simulation.m  
RK4 simulation of the rigid‑body model.

Utilities used:  
quat_kinematics.m, quat_error.m, quat_multiply.m

🟩 Task 1 — Reaction Wheel Momentum Envelope
Goal: Compute and visualize the reaction wheel momentum envelope.

Scripts:

task1_envelope.m  
Computes RW envelope vertices and simulates momentum trajectory.

plot_momentum_envelope.m  
3D visualization of envelope + trajectory.

🟧 Task 2 — VSCMG Geometry and Steering Matrices
Goal: Implement geometric relationships for VSCMGs.

Scripts:

vscmg_geometry.m  
Returns gimbal axes g and spin axes s.

vscmg_spin_axes.m  
Computes spin axes h(δ) as a function of gimbal angles.

vscmg_steering_matrices.m  
Computes steering matrices S and G.

These functions are used in Tasks 4 and 5.

🟨 Task 3 — PD+I Attitude Control (Rigid Body Only)
Goal: Design and test a quaternion PD+I controller on the rigid‑body model.

Scripts:

task3_PD_I.m  
Implements:

Quaternion error feedback

PD+I control law

RK4 integration

No RW or VSCMG actuation

Utilities used:  
Controller_PD_I.m, quat_error.m, attitude_dynamics_7state.m

🟫 Task 4 — VSCMG Steering Law Demonstration
Goal: Demonstrate SRS (singularity‑robust steering) and null‑motion.

Scripts:

task4_steering_demo.m (to be added)  
Will show:

Steering matrices S and G

Combined matrix A = [S G]

σ_min(A) behavior

Null‑motion effect

🟥 Task 5(a) — 60° Slew About Non‑Principal Axis + >80% Momentum Envelope
Goal: Perform a large‑angle slew using RW + VSCMG steering, ensuring momentum reaches >80% of the RW envelope.

Scripts:

task5a_fixed.m  
Implements:

Non‑principal axis reference

RW/VSCMG torque split

SRS + null‑motion

Momentum tracking

α(t) tracking

15‑state dynamics

Dynamics used:  
attitude_dynamics_vscmg.m

🟥 Task 5(b) — Near‑Singular Tracking
Goal: Start near a VSCMG singularity and track a reference while monitoring σ_min(A).

Scripts:

task5b_near_singular_tracking.m  
Implements:

Near‑singular initial gimbal angles

SRS

Null‑motion

σ_min(A) logging

α(t) logging

Attitude error tracking

Dynamics used:  
attitude_dynamics_vscmg.m
