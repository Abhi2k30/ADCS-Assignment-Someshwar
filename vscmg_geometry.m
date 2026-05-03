function [g, s] = vscmg_geometry()
% 4‑VSCMG pyramid geometry (Schaub & Junkins standard)

% Pyramid half-angle (typical: 54.7356 deg)
gamma = deg2rad(54.7356);

% Gimbal axes (pointing outward)
g = [  sin(gamma)   -sin(gamma)   -sin(gamma)    sin(gamma);
       sin(gamma)    sin(gamma)   -sin(gamma)   -sin(gamma);
       cos(gamma)    cos(gamma)    cos(gamma)    cos(gamma) ];

% Spin axes at zero gimbal (orthogonal to g_i)
s = [  cos(gamma)   -cos(gamma)   -cos(gamma)    cos(gamma);
       cos(gamma)    cos(gamma)   -cos(gamma)   -cos(gamma);
      -sin(gamma)   -sin(gamma)   -sin(gamma)   -sin(gamma) ];
end
