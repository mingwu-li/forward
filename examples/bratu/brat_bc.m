function [data, y] = brat_bc(prob, data, u) %#ok<INUSL>

T  = u(1);
x0 = u(2:3);
x1 = u(4:5);

y = [T-1; x0(1); x1(1)];

end
