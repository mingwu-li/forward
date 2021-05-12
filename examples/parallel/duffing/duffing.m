function y = duffing(t,x, p)

x1 = x(1,:);
x2 = x(2,:);
amp   = p(1,:);
omega = p(2,:);

y(1,:) = x2;
y(2,:) = amp.*cos(omega.*t)-0.2*x2-x1-x1.^3;

end
