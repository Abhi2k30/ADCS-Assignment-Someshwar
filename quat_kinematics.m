function qdot = quat_kinematics(q, w)
% Quaternion kinematics for q = [qv; qs] (scalar last)

qv = q(1:3);
qs = q(4);

qv_dot = 0.5 * ( qs*w + cross(qv, w) );
qs_dot = -0.5 * ( qv.' * w );

qdot = [qv_dot; qs_dot];
end
