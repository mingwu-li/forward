function y = duffing(t, x, om)

x1 = x(1);
x2 = x(2);

y    = zeros(2,1);
y(1) = x2;
y(2) = 0.5*cos(om*t)-2*0.1*x2-x1-0.2.*x1^3;

end
