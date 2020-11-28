function [data, y] = net_bc(prob, data, u)

x0 = u(1:8);
x1 = u(9:16);
T0 = u(17);
T  = u(18);
om = u(19);

y = [x1-x0; T0; T-2*pi/om];

end
