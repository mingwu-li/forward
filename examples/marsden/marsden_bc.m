function [data, y] = marsden_bc(prob, data, u)

x0 = u(1:3);
x1 = u(4:6);

y = [x1-x0; x0(2)]; % the last equation corresponds a Poincare section

end
