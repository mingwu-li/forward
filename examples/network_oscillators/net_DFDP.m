function J = net_DFDP(t, x, p)
%VDP_DFDP   'coll'-compatible encoding of Jacobian of langford vector field w.r.t. problem parameters.

om = p(1);
a  = p(2);
ep = 0.05;

J = zeros(8,2);
J(5,1) = -t*ep*a*sin(om*t);
J(5,2) = ep*cos(om*t);

% f  = @(x,p) net(x(1), x(2:end), p);
% tx = [ t ; x ];
% Jp = coco_ezDFDP('f(x,p)', f, tx, p);

end
