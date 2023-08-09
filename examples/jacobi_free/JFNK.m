function [x, R] = JFNK(F, x0)

R = F(x0); % initial residual
x = x0; % initialize solution vector
counter = 1; % iteration counter
tol   = 1e-3;
maxit = min(100,numel(x0));
epsilon  = 1e-6;
max_iter = 20;
num_iter = zeros(1,2);
while 1
    j_v_approx = @(v) JV_APPROX(v, F, x);
    [v,~,~,iter] = gmres(j_v_approx, R, [], tol, maxit); % solve for Krylov vector
    x = x-v; % updated solution
    R = F(x);  % new residual
    num_iter = num_iter+iter;
    if counter > max_iter
        error('JFNK method did not converge!');
    end
    if norm(R)<epsilon && norm(v)<epsilon*norm(x)
        break;
    end
    counter = counter + 1; % update iteration counter
end
fprintf('\n The number of newton iterations is: %d \n', counter); % norm of the residual
fprintf(' The number of outer iterations is %d \n', num_iter(1));
fprintf(' The number of inter+outer iterations is %d \n', sum(num_iter));
end

