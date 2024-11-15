clc; clear all;
M = eye(4);
C = diag([-1; 0.8 ;0.8 ;0.8]);
K = [3 -1 0 -1;-1 4 -1 -1;0 -1 2 0;-1 -1 0 3];
ep = 0.05;
N  = @(u,v,p) p(3)*C*v+K*u+[p(3)*u(1)^2*v(1);0;0;0];
Nu = @(u,v,p) K+[2*p(3)*u(1)*v(1) 0  0 0; zeros(3,4)];
Nv = @(u,v,p) p(3)*C+[p(3)*u(1)^2 0 0 0; zeros(3,4)];
Np = @(u,v,p) [zeros(4,2) C*v+[u(1)^2*v(1);0;0;0]];
F  = @(t,p) [p(3)*p(2)*cos(p(1)*t);0;0;0];
Fp = @(t,p) [-p(3)*p(2)*t*sin(p(1)*t) p(3)*cos(p(1)*t) p(2)*cos(p(1)*t);zeros(3,3)];

% Forward simulation using Newmark
p0 = [ 2; 0.4; 0.05 ]; % amp, omega, ep
T0 = 0;
T  = 2*pi/p0(1);
opts.RelTol = 1e-6;
opts.ItMX   = 10;
opts.Nsteps = 50000;
opts.alpha  = 0;
[uend,vend] = Newmark(M,N,Nu,Nv,Np,F,Fp,T0,100*T,zeros(8,1),p0,opts);
x0 = [uend;vend];
opts.Nsteps = 500;
outdof = [1 2 3];
[uend,vend,zt,Et] = Newmark(M,N,Nu,Nv,Np,F,Fp,T0,T,x0,p0,opts,outdof);
x1 = [uend;vend];

figure;
plot(zt(1,:),zt(3,:))

% Continuation of periodic orbit
prob = coco_prob();
prob = coco_set(prob, 'forward', 'autonomous', false, 'order', 2);
prob = coco_set(prob, 'cont', 'h_max', 5);
ode_opts.RelTol  = 1e-8;
ode_opts.Nsteps  = 300;
prob = coco_set(prob,'forward','ode_opts',ode_opts,'ODEsolver',@Newmark);
forward_args = {M, N, Nu, Nv, Np, F, Fp, T0, T, x0, x1, {'omega' 'amp' 'ep'}, p0};
prob1 = ode_isol2forward(prob, '',forward_args{:});

[data, uidx] = coco_get_func_data(prob1, 'forward', 'data', 'uidx');
bc_funcs = {@net_bc, @net_bc_du};
prob1 = coco_add_func(prob1, 'bc', bc_funcs{:}, [], 'zero', 'uidx', ...
  uidx([data.x0_idx, data.x1_idx, data.T0_idx, data.T_idx, data.p_idx(1)]));

% track amplitude of outdof AND stability
ampData = struct();
ampData.dof  = outdof;
ampData.dim  = 2*4;
ampData.M    = M;
ampData.Nhan = N;
ampData.dNdu = Nu;
ampData.dNdv = Nv;
ampData.dNdp = Np;
ampData.Fext = F;
ampData.dFextdp = Fp;
ampData.opts = opts;
numoutdof    = numel(outdof);
ampNames     = cell(1, numoutdof+1);
for k = 1:numoutdof
   ampNames{k} = strcat('amp',num2str(outdof(k))); 
end
ampNames{k+1} = 'stab';
prob1 = coco_add_func(prob1, 'amp', @amplitude, ampData, 'regular', ampNames,...
    'uidx', uidx([data.x0_idx, data.T0_idx, data.T_idx, data.p_idx]));

% continuation
cont_args = {1, [{'omega'} {'amp'} {'ep'} ampNames(:)'], [1.95 2.05]};

coco(prob1, 'net-second-par-om', [], cont_args{:});

%% Switch to the continuation in epsilon
prob2 = ode_forward2forward(prob, '','net-second-par-om',15);
[data, uidx] = coco_get_func_data(prob2, 'forward', 'data', 'uidx');
prob2 = coco_add_func(prob2, 'bc', bc_funcs{:}, [], 'zero', 'uidx', ...
  uidx([data.x0_idx, data.x1_idx, data.T0_idx, data.T_idx, data.p_idx(1)]));
prob2 = coco_add_func(prob2, 'amp', @amplitude, ampData, 'regular', ampNames,...
    'uidx', uidx([data.x0_idx, data.T0_idx, data.T_idx, data.p_idx]));

% continuation
cont_args = {1, [{'ep'} {'omega'} {'amp'} ampNames(:)'], 1e-2*[1 10]};
coco(prob2, 'net-second-par-ep', [], cont_args{:});

%% validation of stability
% stable periodic orbit
sol1 = forward_read_solution('','net-second-par-om',9);
opts.RelTol = 1e-6;
opts.ItMX   = 10;
opts.Nsteps = 5000;
opts.alpha  = 0;
[~,~,zt] = Newmark(M,N,Nu,Nv,Np,F,Fp,T0,10*T,sol1.x0,p0,opts,[1,5]);
figure;
plot(zt(1,:),zt(2,:));


% unstable periodic orbit
sol2 = forward_read_solution('','net-second-par-om',15);
[~,~,zt] = Newmark(M,N,Nu,Nv,Np,F,Fp,T0,10*T,sol2.x0,p0,opts,[1,5]);
figure;
plot(zt(1,:),zt(2,:));
