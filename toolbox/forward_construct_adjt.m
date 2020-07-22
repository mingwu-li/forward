function [prob, data] = forward_construct_adjt(prob, tbid, data, sol)
%FORWARD_CONSTRUCT_ADJT   Add FORWARD adjoint problem.

opt  = data.forward_opt;
seg  = data.forward_seg;

prob = coco_add_adjt(prob, tbid, @adj, @adj_DU, data, 'l0', sol.l0, ...
  'tl0', sol.tl0, 'adim', opt.adim);

if ~isempty(seg.pnames)
  pfid   = coco_get_id(tbid, 'pars');
  dnames = coco_get_id('d', seg.pnames);
  dnames = coco_get_id(tbid, dnames);
  axidx  = coco_get_adjt_data(prob, tbid, 'axidx');
  prob   = coco_add_adjt(prob, pfid, dnames, 'aidx', axidx(opt.p_idx), ...
    'l0', sol.pars_l0, 'tl0', sol.pars_tl0);
end

if seg.autonomous
  t0fid = coco_get_id(tbid, 'T0');
  axidx = coco_get_adjt_data(prob, tbid, 'axidx');
  dT0   = coco_get_id('d', t0fid);
  prob  = coco_add_adjt(prob, t0fid, dT0, 'aidx', axidx(opt.T0_idx), ...
      'l0', sol.T0_l0, 'tl0', sol.T0_tl0);
end

end

function [data, J] = adj(prob, data, u)
seg = data.forward_seg;

x0 = u(seg.x0_idx);
x1 = u(seg.x1_idx);
T0 = u(seg.T0_idx);
T  = u(seg.T_idx);
p  = u(seg.p_idx);


m  = seg.dim;
n  = seg.pdim;
if seg.autonomous
  f0 = seg.f(x0, p);
else
  f0 = seg.f(T0, x0, p);
end
M0 = [x0 eye(m,m) -f0 zeros(m,n)]; % x, \partial_x0, \partial_T0, \partial_p
f  = @(t,M) VarEQN(seg, m, n, t, M, p);

[~, x] = seg.ODEsolver(f, [T0 T0+T], M0(:), seg.ode_opts);

M1 = x(end,:);
% y  = M1(1:m)' - x1;
Jx = reshape(M1(m+1:m+m*m), m, m);
JT0 = M1(m+m*m+1:2*m+m*m)';
if seg.autonomous
    JT  = seg.f(M1(1:m)', p);
else
    JT  = seg.f(T0+T, M1(1:m)', p);
end
JT0 = JT0 + JT;
Jp  = reshape(M1(2*m+m*m+1:end), m, n);
J   = [ Jx -eye(m,m) JT0 JT Jp ];
%   tic
%   [~,Jnumer] = coco_ezDFDX('f(o,d,x)', prob, data, @forwardy, u);
%   toc
%   max(abs(J(:)-Jnumer(:)))  
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


function [data, dJ] = adj_DU(prob, data, u)
% tic
seg = data.forward_seg;
opt = data.forward_opt;

x0 = u(seg.x0_idx);
T0 = u(seg.T0_idx);
T  = u(seg.T_idx);
p  = u(seg.p_idx);

m  = seg.dim;
n  = seg.pdim;

if seg.autonomous
  f0  = seg.f(x0, p);
  ft0 = zeros(m,1);
  fx0 = seg.fx(x0, p);
  fp0 = seg.fp(x0, p);
else
  f0  = seg.f(T0, x0, p);
  if isempty(seg.ft)
      f   = @(x,p) seg.f(x(1,:), p(1:m,:), p(m+1:end,:));
      xp  = [ x0 ; p ];
      ft0 = coco_ezDFDX('f(x,p)', f, T0, xp);
  else
      ft0 = seg.ft(T0, x0, p);
  end
  fx0 = seg.fx(T0, x0, p);
  fp0 = seg.fp(T0, x0, p);
end
% x, _x0, _T0, _p, _T0T0, _T0p, _x0T0, _x0x0, _x0p, _pp
M0 = [x0 eye(m,m) -f0 zeros(m,n) -ft0+fx0*f0 -fp0 -fx0 zeros(m,m*m) zeros(m,m*n) zeros(m, n*n)]; 
f  = @(t,M) VarEQN2(data, m, n, t, M, p);

[~, xx] = seg.ODEsolver(f, [T0 T0+T], M0(:), seg.ode_opts);
xx   = xx(end,:)';
var2 = opt.var2;
Phi      = xx(var2.phi);
Phi_x0   = reshape(xx(var2.x0), m, m);
Phi_T0   = xx(var2.T0);
Phi_p    = reshape(xx(var2.p), m, n);
Phi_T0T0 = xx(var2.T0T0);
Phi_T0p  = reshape(xx(var2.T0p), m, n);
Phi_x0T0 = reshape(xx(var2.x0T0), m, m);
Phi_x0x0 = reshape(xx(var2.x0x0), m, m, m);
Phi_x0p  = reshape(xx(var2.x0p), m, m, n);
Phi_pp   = reshape(xx(var2.pp), m, n, n);

if seg.autonomous
  F   = seg.f(Phi, p);
  Fx  = seg.fx(Phi, p);
  Fp  = seg.fp(Phi, p);
  Ft  = zeros(m,1);
else
  F   = seg.f(T0+T, Phi, p);
  Fx  = seg.fx(T0+T, Phi, p);
  Fp  = seg.fp(T0+T, Phi, p);
  if isempty(seg.ft)
      f  = @(x,p) seg.f(x(1,:), p(1:m,:), p(m+1:end,:));
      xp = [ Phi ; p ];
      Ft = coco_ezDFDX('f(x,p)', f, T0+T, xp);
  else
      Ft = seg.ft(T0+T, Phi, p);
  end
end

% variation in x0
var_x0_x0 = Phi_x0x0;
var_x0_x1 = zeros(m, m, m);
var_x0_T0 = Fx*Phi_x0+Phi_x0T0;
var_x0_T  = Fx*Phi_x0;
var_x0_p  = Phi_x0p;
var_x0    = cat(3, var_x0_x0, var_x0_x1, var_x0_T0, var_x0_T, var_x0_p);

% variation in x1
var_x1    = zeros(m, m, m+m+2+n);

% variation in T0
var_T0_x0 = var_x0_T0;
var_T0_x1 = zeros(m,m);
var_T0_T0 = Ft+Fx*F+2*Fx*Phi_T0+Phi_T0T0;
var_T0_T  = Ft+Fx*F+Fx*Phi_T0;
var_T0_p  = Fx*Phi_p+Fp+Phi_T0p;
var_T0    = [var_T0_x0 var_T0_x1 var_T0_T0 var_T0_T var_T0_p];
var_T0    = reshape(var_T0, [m, 1, m+m+2+n]);

% variation in T
var_T_x0 = var_x0_T;
var_T_x1 = zeros(m,m);
var_T_T0 = var_T0_T;
var_T_T  = Ft+Fx*F;
var_T_p  = Fx*Phi_p+Fp;
var_T    = [var_T_x0 var_T_x1 var_T_T0 var_T_T var_T_p];
var_T    = reshape(var_T, [m, 1, m+m+2+n]);

% variation in p
var_p_x0 = permute(var_x0_p, [1,3,2]);
var_p_x1 = zeros(m,n,m);
var_p_T0 = var_T0_p;
var_p_T  = var_T_p;
var_p_p  = Phi_pp;
var_p    = cat(3, var_p_x0, var_p_x1, var_p_T0, var_p_T, var_p_p);

dJ = [var_x0 var_x1 var_T0 var_T var_p];

% toc
% tic
% [data, ddJ] = coco_ezDFDX('f(o,d,x)', prob, data, @adj, u);
% toc
% diffJ = dJ(:)-ddJ(:);
% max(abs(diffJ))
end


function y = VarEQN2(data, m, n, t, xx, p)

opt = data.forward_opt;
seg = data.forward_seg;
var2 = opt.var2;
Phi      = xx(var2.phi);
Phi_x0   = reshape(xx(var2.x0), m, m);
Phi_T0   = xx(var2.T0);
Phi_p    = reshape(xx(var2.p), m, n);
Phi_T0T0 = xx(var2.T0T0);
Phi_T0p  = reshape(xx(var2.T0p), m, n);
Phi_x0T0 = reshape(xx(var2.x0T0), m, m);
Phi_x0x0 = reshape(xx(var2.x0x0), m, m, m);
Phi_x0p  = reshape(xx(var2.x0p), m, m, n);
Phi_pp   = reshape(xx(var2.pp), m, n, n);

if seg.autonomous
  F   = seg.f(Phi, p);
  Fx  = seg.fx(Phi, p);
  Fp  = seg.fp(Phi, p);
  if isempty(seg.fxx)
      Fxx = coco_ezDFDX('f(x,p)', seg.fx, Phi, p);
  else
      Fxx = seg.fxx(Phi, p);
  end
  if isempty(seg.fxp)
      Fxp = coco_ezDFDP('f(x,p)', seg.fx, Phi, p);
  else
      Fxp = seg.fxp(Phi, p);
  end
  if isempty(seg.fpp)
      Fpp = coco_ezDFDP('f(x,p)', seg.fp, Phi, p);
  else
      Fpp = seg.fpp(Phi, p);
  end
else
  F   = seg.f(t, Phi, p);
  Fx  = seg.fx(t, Phi, p);
  Fp  = seg.fp(t, Phi, p);
  if isempty(seg.fxx)
      f  = @(x,p) seg.fx(p(1,:), x, p(2:end,:));
      tp = [ t ; p ];
      Fxx = coco_ezDFDX('f(x,p)', f, Phi, tp);
  else
      Fxx = seg.fxx(t, Phi, p);
  end
  if isempty(seg.fxp)
      f  = @(x,p) seg.fx(x(1,:), x(2:end,:), p);
      tx = [ t ; Phi ];
      Fxp = coco_ezDFDP('f(x,p)', f, tx, p);
  else
      Fxp = seg.fxp(t, Phi, p);
  end
  if isempty(seg.fpp)
      f  = @(x,p) seg.fp(x(1,:), x(2:end,:), p);
      tx = [ t ; Phi ];
      Fpp = coco_ezDFDP('f(x,p)', f, tx, p);
  else
      Fpp = seg.fpp(t, Phi, p);
  end  
%   Fxx = data.fxx(t, Phi, p);
%   Fxp = data.fxp(t, Phi, p);
%   Fpp = data.fpp(t, Phi, p);
%   Ft  = data.ft(t, Phi, p);
%   Fxt = data.fxt(t, Phi, p);
%   Fpt = data.fpt(t, Phi, p);
%   Ftt = data.ftt(t, Phi, p);
end

yPhi      = F;
yPhi_x0   = Fx*Phi_x0;
yPhi_T0   = Fx*Phi_T0;
yPhi_p    = Fx*Phi_p+Fp;

if m==1
    yPhi_T0T0 = Fxx*Phi_T0*Phi_T0 + Fx*Phi_T0T0;
    yPhi_T0p  = Fxx*Phi_T0*Phi_p + Fxp*Phi_T0 + Fx*Phi_T0p;
    yPhi_x0T0 = Fxx*Phi_T0*Phi_x0 + Fx*Phi_x0T0;
    yPhi_x0x0 = Fxx*Phi_x0*Phi_x0 + Fx*Phi_x0x0;
    yPhi_x0p  = Fxx*Phi_x0*Phi_p + Fxp*Phi_x0 + Fx*reshape(Phi_x0p,[1,n]);
    yPhi_pp   = Fxx*(Phi_p'*Phi_p) + Phi_p'*Fxp + Fx*reshape(Phi_pp,[n,n]) + Fxp'*Phi_p + reshape(Fpp,[n,n]);    
else
    
% --- original einsum ---
% yPhi_T0T0 = einsum(einsum(Fxx, Phi_T0, 2, 1), Phi_T0, 2 ,1) + Fx*Phi_T0T0;
% yPhi_T0p  = einsum(einsum(Fxx, Phi_T0, 2, 1), Phi_p, 2, 1) + ...
%     einsum(Fxp, Phi_T0, 2, 1) + Fx*Phi_T0p;
% yPhi_x0T0 = einsum(einsum(Fxx, Phi_T0, 2, 1), Phi_x0, 2, 1) + Fx*Phi_x0T0;
% yPhi_x0x0 = einsum(einsum(Fxx, Phi_x0, 2, 1), Phi_x0, 2, 1) + ...
%     einsum(Fx,Phi_x0x0, 2, 1);
% yPhi_x0p  = einsum(einsum(Fxx, Phi_x0, 2, 1), Phi_p, 2, 1) + ...
%     permute(einsum(Fxp, Phi_x0, 2, 1), [1 3 2]) + ...
%     einsum(Fx, Phi_x0p, 2, 1);
% yPhi_pp   = einsum(einsum(Fxx, Phi_p, 2, 1), Phi_p, 2, 1) + ...
%     permute(einsum(Fxp, Phi_p, 2, 1), [1 3 2]) + ...
%     einsum(Fx, Phi_pp, 2, 1) + einsum(Fxp, Phi_p, 2, 1) + Fpp;

% --- einsum with precomputed dimsA and dimsB
% einsum_dims(A,B,iA,iB,dimsA,dimsB)
% dimsA=setdiff(1:ndims(A),iA);
% dimsB=setdiff(1:ndims(B),iB);
yPhi_T0T0 = einsum_dims(einsum_dims(Fxx, Phi_T0, 2, 1, [1,3], 2), Phi_T0, 2, 1, 1, 2) + Fx*Phi_T0T0;
yPhi_T0p  = einsum_dims(einsum_dims(Fxx, Phi_T0, 2, 1, [1,3], 2), Phi_p, 2, 1, 1, 2) + ...
    einsum_dims(Fxp, Phi_T0, 2, 1, [1,3], 2) + Fx*Phi_T0p;
yPhi_x0T0 = einsum_dims(einsum_dims(Fxx, Phi_T0, 2, 1, [1,3], 2), Phi_x0, 2, 1, 1, 2) + Fx*Phi_x0T0;
yPhi_x0x0 = einsum_dims(einsum_dims(Fxx, Phi_x0, 2, 1, [1,3], 2), Phi_x0, 2, 1, [1,3], 2) + ...
    einsum_dims(Fx,Phi_x0x0, 2, 1, 1, [2,3]);
yPhi_x0p  = einsum_dims(einsum_dims(Fxx, Phi_x0, 2, 1, [1,3], 2), Phi_p, 2, 1, [1,3], 2) + ...
    permute(einsum_dims(Fxp, Phi_x0, 2, 1, [1,3], 2), [1 3 2]) + ...
    einsum_dims(Fx, Phi_x0p, 2, 1, 1, [2,3]);
yPhi_pp   = einsum_dims(einsum_dims(Fxx, Phi_p, 2, 1, [1,3], 2), Phi_p, 2, 1, [1,3], 2) + ...
    permute(einsum_dims(Fxp, Phi_p, 2, 1, [1,3], 2), [1 3 2]) + ...
    einsum_dims(Fx, Phi_pp, 2, 1, 1, [2,3]) + einsum_dims(Fxp, Phi_p, 2, 1, [1,3], 2) + Fpp;

end
y  = [ yPhi(:); yPhi_x0(:); yPhi_T0(:); yPhi_p(:)
    yPhi_T0T0(:); yPhi_T0p(:); yPhi_x0T0(:)
    yPhi_x0x0(:); yPhi_x0p(:); yPhi_pp(:)];
% y  = y(:);
end
