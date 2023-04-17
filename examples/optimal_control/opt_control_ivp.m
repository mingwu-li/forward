function y=opt_control_ivp(nterm,varargin)

global ncheb tT
ncheb = nterm; tT=1;

%% first run to find initial fold

T0 = -1;
T  = 2;
x0 = 1;
% zero problems
p0 = zeros(ncheb,1);
par_args = cell(1,ncheb);   
for j=1:ncheb
    par_args{j} = sprintf('p%d',j);
end
[t, x]  = ode45(@(t,x) optcont(t, x, p0), [T0 T0+T], 1);
x1 = x(end);

prob = coco_prob();
prob = coco_set(prob, 'forward', 'autonomous', false);
forward_args = {@optcont, @optcont_dx, @optcont_dp, T0, T, x0, x1, par_args, p0};
prob1 = ode_isol2forward(prob, '', forward_args{:});
[data, uidx] = coco_get_func_data(prob1, 'forward', 'data', 'uidx');
prob1 = coco_add_func(prob1, 'bc', @optcont_bc, @optcont_bc_du, ...
  @optcont_bc_dudu, data, 'zero', 'uidx', ...
  uidx([data.x0_idx, data.T0_idx, data.T_idx]));
prob1 = coco_add_pars(prob1, 'mayer', uidx(data.x1_idx), 'yf');
% adjoints
prob1 = adjt_isol2forward(prob1, '');
[data, axidx] = coco_get_adjt_data(prob1, 'forward', 'data', 'axidx');
opt = data.forward_opt;
prob1 = coco_add_adjt(prob1, 'bc', 'aidx', ...
  axidx([opt.x0_idx; opt.T0_idx; opt.T_idx]));

prob1 = coco_add_adjt(prob1, 'mayer', 'd.yf', 'aidx', axidx(opt.x1_idx));
cont_args = cell(1,ncheb+2);
cont_args{1} = 'yf';
cont_args{2} = 'd.yf';
cont_args{3} = sprintf('p%d',1);

for kid=1:ncheb-1
    cont_args{3+kid} = sprintf('forward.d.p%d',kid+1);
end
coco(prob1, 'optcont1', [], 1, cont_args, [-0.1 0.1]);


%% branch switch from fold to grow nontrivial adjoint
bd1   = coco_bd_read('optcont1');
BPlab = coco_bd_labs(bd1, 'BP');
lab   = BPlab(1);

% zero problems
prob2 = ode_BP2forward(prob, '', 'optcont1', lab); 

[data, uidx] = coco_get_func_data(prob2, 'forward', 'data', 'uidx');

prob2 = coco_add_func(prob2, 'bc', @optcont_bc, @optcont_bc_du, ...
  @optcont_bc_dudu, data, 'zero', 'uidx', ...
  uidx([data.x0_idx, data.T0_idx, data.T_idx])); 

prob2 = coco_add_pars(prob2, 'mayer', uidx(data.x1_idx), 'yf');

% branch switch data
chart = coco_read_solution('optcont1', lab, 'chart');
cdata = coco_get_chart_data(chart, 'lsol');

% adjoints
prob2 = adjt_BP2forward(prob2, '', 'optcont1', lab);

[data, axidx] = coco_get_adjt_data(prob2, 'forward', 'data', 'axidx');
opt = data.forward_opt;

[chart, aidx] = coco_read_adjoint('bc', 'optcont1', lab, 'chart', 'lidx');
prob2   = coco_add_adjt(prob2, 'bc', 'aidx', ...
  axidx([opt.x0_idx; opt.T0_idx; opt.T_idx]), ...
  'l0', chart.x, 'tl0', cdata.v(aidx));
[chart, aidx] = coco_read_adjoint('mayer', 'optcont1', lab, 'chart', 'lidx');
prob2 = coco_add_adjt(prob2, 'mayer', 'd.yf', 'aidx', ...
  axidx(opt.x1_idx), 'l0', chart.x, 'tl0', cdata.v(aidx));

% computational domain
dyf_int = [chart.x(1) 1.1];
prob2 = coco_add_event(prob2, 'opt', 'BP', 'd.yf', '>', 1);

ss = cont_args{1};
cont_args{1} = cont_args{2};
cont_args{2} = ss;
coco(prob2, 'optcont2', [], cont_args, dyf_int);


%% continue to let d.p1=0
for k=1:ncheb-1
prun = sprintf('optcont%d',k+1); % previous run
crun = sprintf('optcont%d',k+2); % current run
bd2 = coco_bd_read(prun);
lab = coco_bd_labs(bd2, 'opt');

% zero problems
prob3 = ode_forward2forward(prob, '', prun, lab); 

[data, uidx] = coco_get_func_data(prob3, 'forward', 'data', 'uidx');

prob3 = coco_add_func(prob3, 'bc', @optcont_bc, @optcont_bc_du, ...
  @optcont_bc_dudu, data, 'zero', 'uidx', ...
  uidx([data.x0_idx, data.T0_idx, data.T_idx])); 

prob3 = coco_add_pars(prob3, 'mayer', uidx(data.x1_idx), 'yf');

% adjoints
prob3 = adjt_forward2forward(prob3, '', prun, lab);

chart = coco_read_adjoint('bc', prun, lab, 'chart');
[data, axidx] = coco_get_adjt_data(prob3, 'forward', 'data', 'axidx');
opt = data.forward_opt;

prob3   = coco_add_adjt(prob3, 'bc', 'aidx', ...
  axidx([opt.x0_idx, opt.T0_idx, opt.T_idx]), 'l0', chart.x);
chart = coco_read_adjoint('mayer', prun, lab, 'chart');
prob3 = coco_add_adjt(prob3, 'mayer', 'd.yf', 'aidx', ...
  axidx(opt.x1_idx), 'l0', chart.x);

% computational domain
chart = coco_read_adjoint('forward.pars', prun, lab, 'chart');
dp10 = chart.x(k+1);
dmu = sprintf('forward.d.p%d',k+1);
if dp10>0
  dp1_int = [-0.1 dp10];
  prob3 = coco_add_event(prob3, 'opt', 'BP', dmu, '<', 0);
else
  dp1_int = [dp10 0.1];
  prob3 = coco_add_event(prob3, 'opt', 'BP', dmu, '>', 0);
end

cont_args = cell(1,ncheb+1);
cont_args{1} = dmu;
cont_args{2} = 'yf';
for kid=1:k+1
    cont_args{2+kid} = sprintf('p%d',kid);
end
for kid=k+2:ncheb
    cont_args{2+kid} = sprintf('forward.d.p%d',kid);
end
coco(prob3, crun, [], cont_args, dp1_int);
end


%% plot optimal results
%% numerical result
bd6 = coco_bd_read(crun);
% control input
p = coco_bd_col(bd6, par_args);
idx = coco_bd_idxs(bd6, 'opt');
popt = p(:,idx);
tp = (0:0.01:2)'; % physical domain [0,2]
t = tp-1;         % standard domain [-1,1]
u = control_expansion(t,popt);

figure(1)
plot(tp,u,'r-.'); hold on
% state variable
lab = coco_bd_labs(bd6, 'opt');
sol = forward_read_solution('', crun, lab);
% figure(2)
% plot(sol.tbp+1,x,'r-.'); hold on

Q = integral(@(x)error_u(x,popt),0,2);
y1 = Q^0.5;

%% analytical result
xe = 4./(1+3*exp(5));                                   % state
y2 = abs(xe-sol.x1)/xe;
y=[y1;y2];

if numel(varargin)>0
    xe = 4./(1+3*exp(2.5*tp));
    ue = xe/2;                                                   % control input
    figure(11); hold on
    plot(tp,u,'ro');
    plot(tp,ue,'b-'); 
    xlabel('t');
    ylabel('control input')
    legend('numerical','exact');
end


end

function y = optcont(t, x, p)

global ncheb tT
u = 0;
for j=1:ncheb
    u = u + p(j,:).*mychebyshevT(j-1,t);
end
y = 2.5*(-x+x.*u-u.^2);
y = tT*y;

end

function J = optcont_dx(t, x, p) 

global ncheb tT
J = zeros(1,1,numel(t));
u = 0;
for j=1:ncheb
    u = u + p(j,:).*mychebyshevT(j-1,t);
end
J(1,1,:) = 2.5*(-1+u);
J = J*tT;

end

function J = optcont_dp(t, x, p) 

global ncheb tT
u = 0;
for j=1:ncheb
    u = u + p(j,:).*mychebyshevT(j-1,t);
end
dydu = 2.5*(x-2*u);
J = zeros(1,ncheb,numel(t));
for j=1:ncheb
    J(1,j,:) = dydu.*mychebyshevT(j-1,t);
end
J  = tT*J;

end

function J = optcont_dt(t, x, p) 

global ncheb tT
u = 0;
for j=1:ncheb
    u = u + p(j,:).*mychebyshevT(j-1,t);
end
dudt = 0;
for j=2:ncheb
    dudt = dudt + p(j,:).*dchebyshevT(j-1,t);
end
dydu = 2.5*(x-2*u);
J = zeros(1,numel(t));
J(1,:) = dydu.*dudt;
J  = tT*J;

end

function dJ = optcont_dxdx(t, x, p) %#ok<INUSD>
dJ = zeros(1,1,1,numel(t));
end

function dJ = optcont_dxdp(t, x, p) %#ok<INUSD>

global ncheb tT
dJ = zeros(1,1,ncheb,numel(t));
for j=1:ncheb
    dJ(1,1,j,:) = 2.5*mychebyshevT(j-1,t);
end
dJ = dJ*tT;

end

function dJ = optcont_dpdp(t, x, p) 

global ncheb tT
dJ = zeros(1,ncheb,ncheb,numel(t));
for i=1:ncheb
    for j=1:ncheb
        dJ(1,i,j,:) = -5*mychebyshevT(j-1,t).*mychebyshevT(i-1,t);
    end
end
dJ  = tT*dJ;

end

function dJ = optcont_dtdx(t, x, p) 

global ncheb tT
dudt = 0;
for j=2:ncheb
    dudt = dudt + p(j,:).*dchebyshevT(j-1,t);
end
dJ = zeros(1,1,numel(t));
dJ(1,1,:) = 2.5*dudt;
dJ  = tT*dJ;

end

function dJ = optcont_dtdp(t, x, p)

global ncheb tT
u = 0;
for j=1:ncheb
    u = u + p(j,:).*mychebyshevT(j-1,t);
end
dudt = 0;
for j=2:ncheb
    dudt = dudt + p(j,:).*dchebyshevT(j-1,t);
end
dydu = 2.5*(x-2*u);
dJ = zeros(1,ncheb,numel(t));
dJ(1,1,:) = -5*mychebyshevT(j-1,t).*dudt;
for j=2:ncheb
    dJ(1,j,:) = -5*mychebyshevT(j-1,t).*dudt+dydu.*dchebyshevT(j-1,t);
end
dJ  = tT*dJ;

end

function dJ = optcont_dtdt(t, x, p)

global ncheb tT
u = 0;
for j=1:ncheb
    u = u + p(j,:).*mychebyshevT(j-1,t);
end
dudt = 0;
for j=2:ncheb
    dudt = dudt + p(j,:).*dchebyshevT(j-1,t);
end
dudtdt = 0;
for j=3:ncheb
    dudtdt = dudtdt + p(j,:).*ddchebyshevT(j-1,t);
end

dydu = 2.5*(x-2*u);
dydudt = -5*dudt;
dJ = zeros(1,numel(t));
dJ(1,:) = dydudt.*dudt+dydu.*dudtdt;
dJ = tT*dJ;

end

function [data, y] = optcont_bc(prob, data, u) %#ok<INUSL>

x0 = u(1);
T0 = u(2);
T  = u(3);

y = [x0-1; T0+1; T-2];% periodicity in R2 x S1 on Poincare section

end

function [data, J] = optcont_bc_du(prob, data, u) %#ok<INUSD,INUSL>

J = eye(3);

end

function [data, dJ] = optcont_bc_dudu(prob, data, u) %#ok<INUSD,INUSL>

dJ = zeros(3,3,3);

end

