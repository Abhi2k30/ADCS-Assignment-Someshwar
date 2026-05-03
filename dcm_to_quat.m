function q = dcm_to_quat(C)
% Converts a DCM to a quaternion [q1 q2 q3 q4]' with q4 = scalar part

tr = trace(C);

if tr > 0
    S = sqrt(tr + 1.0) * 2;
    qw = 0.25 * S;
    qx = (C(3,2) - C(2,3)) / S;
    qy = (C(1,3) - C(3,1)) / S;
    qz = (C(2,1) - C(1,2)) / S;
else
    if (C(1,1) > C(2,2)) && (C(1,1) > C(3,3))
        S = sqrt(1.0 + C(1,1) - C(2,2) - C(3,3)) * 2;
        qw = (C(3,2) - C(2,3)) / S;
        qx = 0.25 * S;
        qy = (C(1,2) + C(2,1)) / S;
        qz = (C(1,3) + C(3,1)) / S;
    elseif (C(2,2) > C(3,3))
        S = sqrt(1.0 + C(2,2) - C(1,1) - C(3,3)) * 2;
        qw = (C(1,3) - C(3,1)) / S;
        qx = (C(1,2) + C(2,1)) / S;
        qy = 0.25 * S;
        qz = (C(2,3) + C(3,2)) / S;
    else
        S = sqrt(1.0 + C(3,3) - C(1,1) - C(2,2)) * 2;
        qw = (C(2,1) - C(1,2)) / S;
        qx = (C(1,3) + C(3,1)) / S;
        qy = (C(2,3) + C(3,2)) / S;
        qz = 0.25 * S;
    end
end

q = [qx; qy; qz; qw];
q = q / norm(q);
end
