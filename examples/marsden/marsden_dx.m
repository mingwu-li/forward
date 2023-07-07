function J = marsden_dx(x, p)
%MARSDEN_DX   jacobian of marsden vector field w.r.t x.

x1 = x(1);
x2 = x(2);
x3 = x(3);
p1 = p(1);
p2 = p(2);

J = zeros(3);
J(1,1) = p1+2*p2*x1;
J(1,2) = 1;
J(2,1) = -1;
J(2,2) = p1+x3;
J(2,3) = x2;
J(3,1) = -1+2*x1;
J(3,2) = p1^2-1;
J(3,3) = -1;

end
