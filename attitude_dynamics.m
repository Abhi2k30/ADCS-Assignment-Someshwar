function xdot = attitude_dynamics(~, x, J, Jinv, tau_act, tau_dist)
% x = [q; w], q (4x1), w (3x1)

q = x(1:4);
w = x(5:7);

% Quaternion kinematics
Omega = quat_omega_matrix(w);
qdot  = 0.5 * Omega * q;

% Rotational dynamics: J*w_dot = -w x (J*w) + tau_act + tau_dist
Hw    = J * w;
wdot  = Jinv * ( -cross(w, Hw) + tau_act + tau_dist );

xdot = [qdot; wdot];
end
