%% Task 3 – PD+I attitude control on rigid body (no RW, no VSCMG)
clear; clc; close all;

%% Inertia matrix (body frame)
J = [ 9.833   -0.06692  -0.05295;
     -0.06692 14.11     -0.002176;
     -0.05295 -0.002176 16.01   ];
Jinv = inv(J);

%% Initial conditions (scalar-last quaternion)
q0 = [0; 0; 0; 1];          % initial attitude
w0 = [0.1; 0.05; -0.02];    % initial body rates [rad/s]
x0 = [q0; w0];              % x = [q; w]

%% Reference attitude: 60 deg about y-axis (can change axis if needed)
axis_ref  = [0; 1; 0];
angle_ref = deg2rad(60);
q_ref     = axisangle2quat(axis_ref, angle_ref);
w_ref     = [0; 0; 0];

%% PD+I gains
Kp = 8*eye(3);
Kd = 4*eye(3);
Ki = 0.05*eye(3);

%% Simulation settings
dt = 0.02;
tf = 200;
N  = round(tf/dt) + 1;
time = linspace(0, tf, N);

x   = zeros(7, N);
x(:,1) = x0;

zI      = zeros(3,1);   % integral state
zI_hist = zeros(3, N);
tau_hist = zeros(3, N);
err_deg = zeros(1, N);

%% Main integration loop (RK4)
for k = 1:N-1
    q = x(1:4, k);
    w = x(5:7, k);

    % --- PD+I attitude controller (body torque) ---
    tau_cmd = Controller_PD_I(q, w, q_ref, w_ref, zI, Kp, Kd, Ki);
    tau_act  = tau_cmd;
    tau_dist = [0; 0; 0];

    % Store control and error
    tau_hist(:,k) = tau_cmd;
    qe = quat_error(q, q_ref);
    err_deg(k) = quat_to_angle_deg(qe);

    % --- RK4 integration of 7-state rigid-body dynamics ---
    k1 = attitude_dynamics(0, x(:,k),           J, Jinv, tau_act, tau_dist);
    k2 = attitude_dynamics(0, x(:,k)+dt/2*k1,   J, Jinv, tau_act, tau_dist);
    k3 = attitude_dynamics(0, x(:,k)+dt/2*k2,   J, Jinv, tau_act, tau_dist);
    k4 = attitude_dynamics(0, x(:,k)+dt*k3,     J, Jinv, tau_act, tau_dist);

    x(:,k+1) = x(:,k) + dt/6*(k1 + 2*k2 + 2*k3 + k4);

    % Normalize quaternion
    x(1:4,k+1) = x(1:4,k+1) / norm(x(1:4,k+1));

    % Integral update
    zI = zI + dt * qe(1:3);
    zI_hist(:,k+1) = zI;
end

% Final error
qe = quat_error(x(1:4,end), q_ref);
err_deg(end) = quat_to_angle_deg(qe);

%% Extract histories
q_hist = x(1:4, :);
w_hist = x(5:7, :);

%% Plots
figure;

% Quaternion
subplot(3,1,1);
plot(time, q_hist, 'LineWidth', 1.5);
legend('q_1','q_2','q_3','q_4','Location','best');
xlabel('Time [s]'); ylabel('q'); grid on;
title('Quaternion (scalar-last)');

% Body rates
subplot(3,1,2);
plot(time, w_hist, 'LineWidth', 1.5);
legend('\omega_x','\omega_y','\omega_z','Location','best');
xlabel('Time [s]'); ylabel('\omega [rad/s]'); grid on;
title('Body rates');

% Attitude error
subplot(3,1,3);
plot(time, err_deg, 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Error [deg]'); grid on;
title('Attitude error angle');

figure;
plot(time, tau_hist, 'LineWidth', 1.5);
legend('\tau_x','\tau_y','\tau_z','Location','best');
xlabel('Time [s]'); ylabel('Torque [N·m]'); grid on;
title('Control torque history');
    