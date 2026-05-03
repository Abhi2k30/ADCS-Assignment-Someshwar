function [S, G] = vscmg_steering_matrices(h, g, Omega, Js)
% Build S(delta) and G(delta,Omega) steering matrices

N = 4;
S = zeros(3,N);
G = zeros(3,N);

for i = 1:N
    S(:,i) = Js * h(:,i);
    G(:,i) = Js * Omega(i) * cross(g(:,i), h(:,i));
end
end
