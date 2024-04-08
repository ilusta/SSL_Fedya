clear; clc;

syms w1 w2 w3 theta alpha2 alpha3 R r x_i y_i theta_i

theta = 0;

W = [w1;w2;w3];
S = [-sin(theta) cos(theta) R;
    -sin(theta+alpha2) cos(theta+alpha2) R;
    -sin(theta+alpha3) cos(theta+alpha3) R];
C = [cos(theta) 0 0;
    0 cos(theta) 0;
    0 0 1];
V = [x_i;
    y_i;
    theta_i];

% pretty(C^-1*S^-1*W*r);

% W == 1/r*S*C*V;

A = solve(W == 1/r*S*C*V, V);
pretty(simplify(A.x_i))
pretty(simplify(A.y_i))
pretty(simplify(A.theta_i))
% A.x_i