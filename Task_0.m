clear; clc; close all;

%% Inertia matrix (body frame)
J = [ 9.833   -0.06692  -0.05295;
     -0.06692 14.11     -0.002176;
     -0.05295 -0.002176 16.01   ];
Jinv = inv(J);

%% Initial conditions (scalar last)
q0 = [0; 0; 0; 1];                 % Identity quaternion
w0 = [0.1; 0.05; -0.02];           % rad/s
x0 = [q0; w0];

%% Simulation settings
t0   = 0;
tf   = 200;
dt   = 0.02;
N    = round((tf - t0)/dt) + 1;
time = linspace(t0, tf, N);

x = zeros(7, N);
x(:,1) = x0;

%% Main integration loop (RK4)
for k = 1:N-1
    tk = time(k);
    xk = x(:,k);

    tau_act  = [0; 0; 0];   % No actuation
    tau_dist = [0; 0; 0];   % No disturbances

    k1 = attitude_dynamics(tk, xk, J, Jinv, tau_act, tau_dist);
    k2 = attitude_dynamics(tk + dt/2, xk + dt/2 * k1, J, Jinv, tau_act, tau_dist);
    k3 = attitude_dynamics(tk + dt/2, xk + dt/2 * k2, J, Jinv, tau_act, tau_dist);
    k4 = attitude_dynamics(tk + dt,   xk + dt   * k3, J, Jinv, tau_act, tau_dist);

    x(:,k+1) = xk + dt/6 * (k1 + 2*k2 + 2*k3 + k4);

    % Renormalize quaternion (scalar last)
    q = x(1:4, k+1);
    x(1:4, k+1) = q / norm(q);
end

%% Extract results
q_hist = x(1:4, :);
w_hist = x(5:7, :);

H_hist = zeros(3, N);
for k = 1:N
    H_hist(:,k) = J * w_hist(:,k);
end

%% Plots
figure;
subplot(2,1,1);
plot(time, q_hist);
legend('q_1','q_2','q_3','q_4 (scalar)');
xlabel('Time [s]'); ylabel('Quaternion');
grid on; title('Quaternion components');

subplot(2,1,2);
plot(time, w_hist);
legend('\omega_x','\omega_y','\omega_z');
xlabel('Time [s]'); ylabel('Body rates [rad/s]');
grid on; title('Body angular rates');

figure;
plot(time, H_hist);
legend('H_x','H_y','H_z');
xlabel('Time [s]'); ylabel('Angular momentum [N·m·s]');
grid on; title('Body angular momentum');
