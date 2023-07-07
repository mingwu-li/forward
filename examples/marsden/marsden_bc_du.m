function [data, J] = marsden_bc_du(prob, data, u)

J  = zeros(4,6);
J(1:3,1:3) = -eye(3);
J(1:3,4:6) = eye(3);
J(4,2) = 1;

end

