function [data, J] = net_bc_du(prob, data, u)

om = u(19);
J  = zeros(10,19);
J(1:8,1:8)  = -eye(8);
J(1:8,9:16) = eye(8);
J(9,17)  = 1;
J(10,18) = 1;
J(10,19) = 2*pi/om^2;

% [~,Jp] = coco_ezDFDX('f(o,d,x)',prob,data,@net_bc,u);

end
