function q = quat_multiply(p, r)
% Quaternion multiply p ⊗ r, scalar-last

pv = p(1:3); ps = p(4);
rv = r(1:3); rs = r(4);

qv = ps*rv + rs*pv + cross(pv, rv);
qs = ps*rs - pv.'*rv;

q = [qv; qs];
end
