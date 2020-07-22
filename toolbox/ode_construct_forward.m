function prob = ode_construct_forward(prob, tbid, data, sol)
%ODE_CONSTRUCT_FORWARD   Append an instance of 'forward' to problem.
%
% Add collocation and continuity conditions, monitor functions that
% evaluate to the problem parameters, and corresponding inactive
% continuation parameters.
%
% PROB = ODE_CONSTRUCT_FORWARD(PROB, TBID, DATA, SOL)
%
% PROB - Continuation problem structure.
% TBID - Toolbox instance identifier.
% DATA - Toolbox data structure.
% SOL  - Initial solution guess.

prob = coco_add_func(prob, tbid, @forward, data, 'zero', ...
  'u0', sol.u, 'F+dF', 't0', sol.t0);

if ~isempty(data.pnames)
  pfid = coco_get_id(tbid, 'pars');
  uidx = coco_get_func_data(prob, tbid, 'uidx'); % Context-dependent index array
  prob = coco_add_pars(prob, pfid, uidx(data.p_idx), data.pnames);
end

if data.autonomous
  t0fid = coco_get_id(tbid, 'T0');
  uidx  = coco_get_func_data(prob, tbid, 'uidx'); % Context-dependent index array
  prob  = coco_add_pars(prob, t0fid, uidx(data.T0_idx), t0fid);
end

prob = coco_add_slot(prob, tbid, @coco_save_data, data, 'save_full');

end

function [data, y, J] = forward(prob, data, u) %#ok<INUSL>
x0 = u(data.x0_idx);
x1 = u(data.x1_idx);
T0 = u(data.T0_idx);
T  = u(data.T_idx);
p  = u(data.p_idx);

if nargout<3
  if data.autonomous
    f = @(t,x) data.f(x, p);
  else
    f = @(t,x) data.f(t, x, p);
  end
  
  [~, x] = data.ODEsolver(f, [T0 T0+T], x0, data.ode_opts);
  
  y = x(end,:)' - x1;
else
%     tic
  m  = data.dim;
  n  = data.pdim;
  if data.autonomous
      f0 = data.f(x0, p);
  else
      f0 = data.f(T0, x0, p);
  end
  M0 = [x0 eye(m,m) -f0 zeros(m,n)]; % x, \partial_x0, \partial_T0, \partial_p
  f  = @(t,M) VarEQN(data, m, n, t, M, p);
  
  [~, x] = data.ODEsolver(f, [T0 T0+T], M0(:), data.ode_opts);
  
  M1 = x(end,:);
  y  = M1(1:m)' - x1;
  Jx = reshape(M1(m+1:m+m*m), m, m);
  JT0 = M1(m+m*m+1:2*m+m*m)';
  if data.autonomous
    JT  = data.f(M1(1:m)', p);
  else
    JT  = data.f(T0+T, M1(1:m)', p);
  end
  JT0 = JT0 + JT;
  Jp  = reshape(M1(2*m+m*m+1:end), m, n);
  J   = [ Jx -eye(m,m) JT0 JT Jp ];
%   toc
%   
%   tic
%   [~,Jnumer] = coco_ezDFDX('f(o,d,x)', prob, data, @forwardy, u);
%   toc
%   max(abs(J(:)-Jnumer(:)))
  
end

end

function y = VarEQN(data, m, n, t, xx, p)
M  = reshape(xx(m+1:m+m*m), m, m);
L  = xx(m+m*m+1:2*m+m*m);
N  = reshape(xx(2*m+m*m+1:end), m, n);
if data.autonomous
  Fx = data.fx(xx(1:m), p);
  Fp = data.fp(xx(1:m), p);
  y  = [ data.f(xx(1:m), p) Fx*M Fx*L Fx*N+Fp ];
else
  Fx = data.fx(t, xx(1:m), p);
  Fp = data.fp(t, xx(1:m), p);
  y  = [ data.f(t, xx(1:m), p) Fx*M Fx*L Fx*N+Fp ];
end
y  = y(:);
end


function [data, y] = forwardy(prob, data, u) %#ok<INUSL>
x0 = u(data.x0_idx);
x1 = u(data.x1_idx);
T0 = u(data.T0_idx);
T  = u(data.T_idx);
p  = u(data.p_idx);

if data.autonomous
f = @(t,x) data.f(x, p);
else
f = @(t,x) data.f(t, x, p);
end

[~, x] = data.ODEsolver(f, [T0 T0+T], x0, data.ode_opts);

y = x(end,:)' - x1;

end

