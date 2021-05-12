function y = doedel_seq(x, p)
%DOEDEL   'coll'-compatible encoding of catenary vector field

x1 = x(1,:);
x2 = x(2,:);
x3 = x(3,:);
x4 = x(4,:);
p1 = p(1,:);
p2 = p(2,:);

y(1,:) = 1-x1.^2;
y(2,:) = p1.*x1+p2.*x2;
y(3,:) = 1-x3.^2;
y(4,:) = p1.*x3+p2.*x4;

end
