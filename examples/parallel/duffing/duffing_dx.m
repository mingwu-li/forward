function J = duffing_dx(t, x, p)

x1 = x(1,:);

J = zeros(2,2,numel(x1));
J(1,2,:) = 1;
J(2,1,:) = -1-3*x1.^2;
J(2,2,:) = -0.2;

end
