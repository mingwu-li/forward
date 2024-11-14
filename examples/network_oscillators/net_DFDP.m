function J = net_DFDP(t, x, p)
%VDP_DFDP   'coll'-compatible encoding of Jacobian of langford vector field w.r.t. problem parameters.

u  = x(1:4); % disp
v  = x(5:8); % vel
om = p(1);
a  = p(2);
ep = p(3);
u1 = u(1);
v1 = v(1);

C = diag([-1; 0.8 ;0.8 ;0.8]);
J = zeros(8,2);
J(5,1) = -t*ep*a*sin(om*t);
J(5,2) = ep*cos(om*t);
J(5:8,3) = -C*v;
J(5,3) = J(5,3)+a.*cos(om.*t)-u1.^2.*v1;

% f  = @(x,p) net(x(1), x(2:end), p);
% tx = [ t ; x ];
% Jp = coco_ezDFDP('f(x,p)', f, tx, p);

end
