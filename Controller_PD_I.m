function tau = Controller_PD_I(q, w, q_ref, w_ref, zI, Kp, Kd, Ki)
qe = quat_error(q, q_ref);
e  = qe(1:3);          % vector part
ew = w - w_ref;

tau = -Kp*e - Kd*ew - Ki*zI;
end
