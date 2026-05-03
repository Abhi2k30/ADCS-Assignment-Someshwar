function R = axis_angle_to_dcm(axis, angle)
g = axis / norm(axis);

gx = g(1); gy = g(2); gz = g(3);
c = cos(angle);
s = sin(angle);
v = 1 - c;

R = [ gx*gx*v + c,     gx*gy*v - gz*s, gx*gz*v + gy*s;
      gy*gx*v + gz*s,  gy*gy*v + c,    gy*gz*v - gx*s;
      gz*gx*v - gy*s,  gz*gy*v + gx*s, gz*gz*v + c    ];
end
