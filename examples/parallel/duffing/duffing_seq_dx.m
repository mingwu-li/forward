function J = duffing_seq_dx(t, x, p)

x1 = x(1,:);
x3 = x(3,:);

J = zeros(4,4,numel(x1));
J(1,2,:) = 1;
J(2,1,:) = -1-3*x1.^2;
J(2,2,:) = -0.2;
J(3,4,:) = 1;
J(4,3,:) = -1-3*x3.^2;
J(4,4,:) = -0.2;

end