function qe = quat_error(q, q_ref)
% Both scalar-last: [qv; qs]
qs = q(4);      qv = q(1:3);
qrs = q_ref(4); qrv = q_ref(1:3);

% Conjugate of reference
q_ref_conj = [-qrv; qrs];

% Quaternion product q_e = q_ref_conj ⊗ q
v1 = q_ref_conj(1:3);
s1 = q_ref_conj(4);
v2 = qv;
s2 = qs;

ve = s1*v2 + s2*v1 + cross(v1,v2);
se = s1*s2 - dot(v1,v2);

qe = [ve; se];
end
