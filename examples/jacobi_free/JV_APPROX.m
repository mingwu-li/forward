function y = JV_APPROX(v, F, x)
% JV_APPROX Finite difference approximation of directional derivative of F
% evaluated at x along v direction.

% determine perturbation amplitude epsilon
dim = size(x, 1);
% if norm(v, 2) > eps 
%     sum = 0;
%     for i = 1 : dim
%         sum = sum + sqrt(eps) * (1 + x(i));
%     end
%     per = (1 / (dim * norm(v, 2))) * sum;
% else
%     sum = 0;
%     for i = 1 : dim
%         sum = sum + sqrt(eps) * (1 + x(i));
%     end
%     per = sum / dim;
% end
b   = sqrt(eps);
if norm(v)>eps    
    per = sum(b*abs(x)+b)/dim/norm(v);
    per = sqrt((1+norm(x))*eps)/norm(v);
else
    per = sum(b*abs(x)+b)/dim;
    per = sqrt((1+norm(x))*eps);
end    

R    = F(x);             % unperturbed residual
xper = x + per * v;   % perturbed vector
Rper = F(xper);       % perturbed residual
y    = (Rper - R) / per; % approximation of jacobian action on krylov vector
end