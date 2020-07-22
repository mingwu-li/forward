function prob = ode_isol2forward_old(prob, fid, f, fx, fp, T0, T, x0, x1, pnames, p0)
% Constructor of forward simulation toolbox.

tbid = coco_get_id(fid, 'forward');

defaults.autonomous = true;
defaults.ODEsolver = @ode45; % use ode45 by default and define solver options
defaults.ode_opts  = odeset('RelTol', 1.0e-8, 'AbsTol', 1.0e-10, 'NormControl', 'on');

copts = coco_get(prob, tbid);
copts = coco_merge(defaults, copts);

fields = {'tbid' 'f' 'fx' 'fp'
  tbid   f   fx   fp};
data        = struct(fields{:});
data.dim    = size(x0,1);
data.x0_idx = 1:numel(x0);
data.x1_idx = data.x0_idx(end) + (1:numel(x1));
data.T0_idx = data.x1_idx(end) + 1;
data.T_idx  = data.T0_idx + 1;
data.p_idx  = data.T_idx + (1:numel(p0));
data.Jxrows = repmat(1:data.dim, [1, data.dim]);
data.Jxcols = repmat(1:data.dim, [data.dim, 1]);
data.JTrows = 1:data.dim;
data.JTcols = ones(1,data.dim);
data.Jprows = repmat(1:data.dim, [1, numel(p0)]);
data.Jpcols = repmat(1:numel(p0), [data.dim 1]);

data.ode_opts  = copts.ode_opts;
data.ODEsolver = copts.ODEsolver;
data.autonomous = copts.autonomous;

data = coco_func_data(data);

prob = coco_add_func(prob, tbid, @forward, data, 'zero', ...
  'u0', [ x0(:) ; x1(:); T0; T; p0 ], 'F+dF');

if ~isempty(pnames)
  pfid = coco_get_id(tbid, 'pars');
  prob = coco_add_pars(prob, pfid, data.p_idx, pnames);
end

if data.autonomous
  prob = coco_add_pars(prob, 'forward.T0', data.T0_idx, 'forward.T0');
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
  m  = numel(x0);
  n  = numel(p);
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
