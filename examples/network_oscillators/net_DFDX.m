function J = net_DFDX(t, x, p)
%VDP_DFDX   'coll'-compatible encoding of Jacobian of langford vector field w.r.t. problem variables

u1 = x(1);
v1 = x(5);

ep = p(3);

C = diag([-1; 0.8 ;0.8 ;0.8]);
K = [3 -1 0 -1;-1 4 -1 -1;0 -1 2 0;-1 -1 0 3];


J = zeros(8,8);
J(1:4,5:8) = eye(4);
J(5:8,1:4) = -K;
J(5:8,5:8) = -ep*C;
J(5,1) = J(5,1)-2*ep*u1*v1;
J(5,5) = J(5,5)-ep*u1^2;

% f  = @(x,p) net(p(1), x, p(2:end));
% tp = [ t ; p ];
% Jx = coco_ezDFDX('f(x,p)', f, x, tp);


end
