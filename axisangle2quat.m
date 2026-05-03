function q = axisangle2quat(axis, angle)
a = axis / norm(axis);
s = sin(angle/2);
qv = a * s;
qs = cos(angle/2);
q  = [qv; qs];   % scalar-last
end
