function [data, y] = forward_map(prob, data, u)
x0 = u(1:end-1);
om = u(end);

% forward simulation with one period
T = 2*pi/om;
odefunc = @(t,x) duffing(t,x,om);
[~,xt] = ode45(odefunc,[0,T],x0);
xf = xt(end,:)';
y = xf-x0;

end