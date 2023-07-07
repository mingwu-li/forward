function J = marsden_dp(x, p)
%MARSDEN_DP   jacobian of marsden vector field w.r.t. p.

x1 = x(1);
x2 = x(2);
x3 = x(3);
p1 = p(1);
p2 = p(2);

J = zeros(3,2);
J(1,1) = x1;
J(1,2) = x1^2;
J(2,1) = x2;
J(3,1) = 2*p1*x2;

end