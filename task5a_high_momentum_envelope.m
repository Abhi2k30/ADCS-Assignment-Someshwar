%% Task 5(a) – 60° slew about non‑principal axis with RW/VSCMG steering
clear; clc; close all;

%% Inertia matrix
J = [ 9.833   -0.06692  -0.05295;
     -0.06692 14.11     -0.002176;
     -0.05295 -0.002176 16.01   ];
Jinv = inv(J);

%% Reaction wheel geometry
A_rw = (1/sqrt(3)) * [ ...
     1   -1   -1    1;
     1    1   -1   -1;
     1    1    1    1 ];

Js_rw  = 0.01;
Js_cmg = 0.01;

%% Initial conditions
q0     = [0;0;0;1];
w0     = [0;0;0];

% Start with high wheel momentum to reach >80% envelope
Omega0 = 50 * ones(4,1);     % adjust if needed
delta0 = zeros(4,1);

x0 = [q0; w0; delta0; Omega0];

%% Reference attitude: 60° about NON‑principal axis
axis  = [1; 1; 0.5];
axis  = axis / norm(axis);
angle = deg2rad(60);
q_ref = axisangle2quat(axis, angle);
w_ref = [0;0;0];

%% Controller gains
Kp = 8*eye(3);
Kd = 4*eye(3);
Ki = 0.05*eye(3);

%% Steering / allocation parameters
tau_th = 0.5;
lambda = 0.1;
K_null = 0.05;

%% Simulation settings
dt = 0.02;
tf = 200;
N  = round(tf/dt)+1;
time = linspace(0, tf, N);

x = zeros(15,N);
x(:,1) = x0;

zI = zeros(3,1);
err = zeros(1,N);
alpha_hist = zeros(1,N);
H_total = zeros(3,N);

%% Main loop
for k = 1:N-1
    q     = x(1:4,k);
    w     = x(5:7,k);
    delta = x(8:11,k);
    Omega = x(12:15,k);

    % Attitude controller
    tau_cmd = Controller_PD_I(q, w, q_ref, w_ref, zI, Kp, Kd, Ki);

    % Torque split
    tau_norm = norm(tau_cmd);
    alpha    = min(1, tau_norm / tau_th);
    tau_rw    = alpha    * tau_cmd;
    tau_vscmg = (1-alpha)* tau_cmd;
    alpha_hist(k) = alpha;

    % RW allocation
    u_rw = pinv(A_rw) * tau_rw;

    % VSCMG steering
    [g, s] = vscmg_geometry();
    h      = vscmg_spin_axes(g, s, delta);
    [S, G] = vscmg_steering_matrices(h, g, Omega, Js_cmg);
    A = [S, G];

    % SRS
    M    = A*A.' + (lambda^2)*eye(3);
    u_sr = A.' * (M \ tau_vscmg);

    % Null motion
    A_pinv = pinv(A);
    N_null = eye(8) - A_pinv * A;
    v_null = [zeros(4,1); -K_null * delta];
    u      = u_sr + N_null * v_null;

    u_Omega = u(1:4);
    u_delta = u(5:8);

    % Integrate dynamics
    k1 = attitude_dynamics_vscmg(0, x(:,k),         J, Jinv, A_rw, Js_cmg, u_rw, u_delta, u_Omega);
    k2 = attitude_dynamics_vscmg(0, x(:,k)+dt/2*k1, J, Jinv, A_rw, Js_cmg, u_rw, u_delta, u_Omega);
    k3 = attitude_dynamics_vscmg(0, x(:,k)+dt/2*k2, J, Jinv, A_rw, Js_cmg, u_rw, u_delta, u_Omega);
    k4 = attitude_dynamics_vscmg(0, x(:,k)+dt*k3,   J, Jinv, A_rw, Js_cmg, u_rw, u_delta, u_Omega);

    x(:,k+1) = x(:,k) + dt/6*(k1+2*k2+2*k3+k4);
    x(1:4,k+1) = x(1:4,k+1)/norm(x(1:4,k+1));

    % Error + integral
    qe = quat_error(q, q_ref);
    err(k) = quat_to_angle_deg(qe);
    zI = zI + dt * qe(1:3);

    % Total momentum (RW + body)
    H_total(:,k) = J*w + A_rw * (Js_rw * Omega);
end

%% Final error
qe = quat_error(x(1:4,end), q_ref);
err(end) = quat_to_angle_deg(qe);

%% Plots
figure;
subplot(3,1,1);
plot(time, err, 'LineWidth',1.5);
title('Attitude Error [deg]'); grid on;

subplot(3,1,2);
plot(time, alpha_hist, 'LineWidth',1.5);
title('\alpha(t) RW/VSCMG Split'); grid on;

subplot(3,1,3);
plot(time, H_total, 'LineWidth',1.5);
title('Total Angular Momentum'); grid on;
xlabel('Time [s]');
