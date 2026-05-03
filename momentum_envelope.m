clear; clc; close all;

%% Inertia / geometry from your main model
J = [ 9.833   -0.06692  -0.05295;
     -0.06692 14.11     -0.002176;
     -0.05295 -0.002176 16.01   ];

A_rw = (1/sqrt(3)) * [ ...
     1   -1   -1    1;
     1    1   -1   -1;
     1    1    1    1 ];

Js_rw   = 0.01;   % example RW inertia
Js_cmg  = 0.01;   % same as in main_task3

Omega_rw_max  = 500;   % rad/s (example)
Omega_cmg_max = 500;   % rad/s (example)
delta_max_deg = 60;
delta_grid_deg = linspace(-delta_max_deg, delta_max_deg, 15);
delta_grid = deg2rad(delta_grid_deg);

%% Preallocate storage
H_tot = [];

[g, s] = vscmg_geometry();

%% Sample envelope
for d1 = delta_grid
for d2 = delta_grid
for d3 = delta_grid
for d4 = delta_grid

    delta = [d1; d2; d3; d4];

    % Spin axes for this gimbal configuration
    h = vscmg_spin_axes(g, s, delta);

    % Extreme wheel speeds (corners of hypercube)
    wheel_signs = dec2bin(0:2^4-1) - '0';  % 16 combinations
    wheel_signs(wheel_signs==0) = -1;

    for k = 1:size(wheel_signs,1)
        sig = wheel_signs(k,:).';

        % RW momentum
        Omega_rw = sig * Omega_rw_max;
        h_rw = A_rw * (Js_rw * Omega_rw);

        % VSCMG momentum
        Omega_cmg = sig * Omega_cmg_max;
        h_cmg = zeros(3,1);
        for i = 1:4
            h_cmg = h_cmg + Js_cmg * Omega_cmg(i) * h(:,i);
        end

        H_tot = [H_tot, h_rw + h_cmg];
    end
end
end
end
end

%% Plot envelope
figure;
plot3(H_tot(1,:), H_tot(2,:), H_tot(3,:), '.', 'MarkerSize', 4);
grid on; axis equal;
xlabel('h_x [Nms]');
ylabel('h_y [Nms]');
zlabel('h_z [Nms]');
title('Combined RW + VSCMG Momentum Envelope (Body Frame)');
