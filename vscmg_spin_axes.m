function h = vscmg_spin_axes(g, s, delta)
% Compute spin axes h_i(delta_i) for all 4 VSCMGs

h = zeros(3,4);
for i = 1:4
    Rg = axis_angle_to_dcm(g(:,i), delta(i));  % rotation about g_i
    h(:,i) = Rg * s(:,i);
end
end
