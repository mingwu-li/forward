function y = duffing_seq(t,x, p)

x1 = x(1,:);
x2 = x(2,:);
x3 = x(3,:);
x4 = x(4,:);
amp   = p(1,:);
omega = p(2,:);

y(1,:) = x2;
y(2,:) = amp.*cos(omega.*t)-0.2*x2-x1-x1.^3;
y(3,:) = x4;
y(4,:) = amp.*cos(omega.*t)-0.2*x4-x3-x3.^3;

end