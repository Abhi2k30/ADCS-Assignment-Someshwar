%% singularity_scan.m
% Internal singular surfaces of the 4‑VSCMG pyramid
% via minimum singular value of A = [S  G].

clear; clc; close all;

%% Parameters
Js        = 0.01;          % CMG wheel inertia
Omega_nom = 300;           % nominal wheel speed for scan
delta_max_deg  = 60;
n_grid          = 40;
delta_grid_deg  = linspace(-delta_max_deg, delta_max_deg, n_grid);
delta_grid      = deg2rad(delta_grid_deg);

%% Geometry
[g, s] = vscmg_geometry();

%% Storage
sig_min_vals  = [];
delta_samples = [];

%% 2‑D gimbal scan: delta = [d1; d2; -d1; -d2]
for d1 = delta_grid
    for d2 = delta_grid

        delta = [ d1;
                  d2;
                 -d1;
                 -d2 ];

        % Spin axes
        h = vscmg_spin_axes(g, s, delta);

        % Steering matrices
        [S, G] = vscmg_steering_matrices(h, g, Omega_nom*ones(4,1), Js);

        % Combined steering matrix
        A = [S, G];   % 3x8

        % Minimum singular value
        svals   = svd(A);
        sig_min = svals(end);

        sig_min_vals  = [sig_min_vals; sig_min];
        delta_samples = [delta_samples; d1, d2];
    end
end

%% Convert to degrees for plotting
delta_deg = delta_samples * 180/pi;

%% Threshold for near-singularity
eps_sing  = 0.05 * max(sig_min_vals);
near_sing = sig_min_vals < eps_sing;

%% Plot σ_min over 2‑D gimbal space
figure;
scatter(delta_deg(:,1), delta_deg(:,2), 20, sig_min_vals, 'filled');
colorbar;
xlabel('\delta_1 [deg]');
ylabel('\delta_2 [deg]');
title('\sigma_{min}(A = [S  G]) over gimbal space');
grid on;

%% Plot near‑singular configurations
figure;
scatter(delta_deg(near_sing,1), delta_deg(near_sing,2), 20, 'r', 'filled');
xlabel('\delta_1 [deg]');
ylabel('\delta_2 [deg]');
title('Near‑singular configurations (internal singular surfaces)');
grid on;
