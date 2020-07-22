function [data, y] = duffing_bc(prob, data, u)  %#ok<INUSD,INUSL>

x0 = u(1:3);
x1 = u(4:6);

y = [x1(1)-x0(1); x1(2)-x0(2); x1(3)-x0(3)-2*pi; x0(2)];
  
end
