function sol = forward_init_sol(T0, T, x0, x1, p0, t0)
%FORWARD_INIT_SOL   Build initial solution guess.
%
% Construct initial solution guess for 'forward' toolbox.
%
% SOL = FORWARD_INIT_SOL(DATA, T0, X0, P0)
%
% T0   - Array of temporal mesh points.
% X0   - Array of state vectors at mesh points.
% P0   - Initial solution guess for problem parameters.

sol.u  = [x0(:); x1(:); T0; T; p0(:)];
sol.t0 = t0;

end
