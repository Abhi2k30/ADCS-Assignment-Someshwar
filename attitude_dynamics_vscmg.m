function xdot = attitude_dynamics_vscmg(~, x, J, Jinv, A_rw, Js_cmg, u_rw, u_delta, u_Omega)
% 15‑state dynamics for RW + 4‑VSCMG system
% x = [q(4); w(3); delta(4); Omega(4)]
% u_rw    : 4x1 RW torque commands (in wheel space)
% u_delta : 4x1 gimbal rate commands [rad/s]
% u_Omega : 4x1 wheel spin acceleration commands [rad/s^2]

    % Unpack state
    q     = x(1:4);
    w     = x(5:7);
    delta = x(8:11);
    Omega = x(12:15);

    % --- RW body torque ---
    % tau_rw_cmd was designed in body frame, then mapped to wheel torques:
    %   u_rw = pinv(A_rw)*tau_rw_cmd
    % So the actual body torque from RW is:
    tau_rw_body = A_rw * u_rw;   % 3x1

    % --- VSCMG torque via S and G ---
    [g, s] = vscmg_geometry();
    h      = vscmg_spin_axes(g, s, delta);
    [S, G] = vscmg_steering_matrices(h, g, Omega, Js_cmg);

    tau_vscmg_body = S * u_Omega + G * u_delta;   % 3x1

    % --- Total body torque ---
    tau_body = tau_rw_body + tau_vscmg_body;

    % --- Attitude kinematics ---
    qdot = quat_kinematics(q, w);

    % --- Rigid‑body dynamics ---
    wdot = Jinv * ( -cross(w, J*w) + tau_body );

    % --- Gimbal and wheel dynamics ---
    deltadot = u_delta;
    Omegadot = u_Omega;

    % --- Pack derivative ---
    xdot = [qdot;
            wdot;
            deltadot;
            Omegadot];
end
