function [data, y] = duffing_bc(prob, data, u)

x0 = u(1:4);
x1 = u(5:8);
T0 = u(9);
T  = u(10);
om = u(11);

y = [x1(:)-x0(:); T0; T-2*pi/om];
end
