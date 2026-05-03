function ang_deg = quat_to_angle_deg(q)
% scalar-last
qs = q(4);
qs = max(min(qs,1),-1);   % clamp for safety
ang = 2*acos(qs);
ang_deg = rad2deg(ang);
end
