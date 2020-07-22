
T0 = 0;
T  = 2*pi;
x0 = [0.276303; 0.960863];
p0 = [0.98; 0.3];

profile on

% forward simulation with ode45
f = @(t,x) linode(t, x, p0);
[t,x] = ode45(f, [T0, T0+T], x0);
x1 = x0;
prob = coco_prob();
prob = coco_set(prob, 'forward', 'autonomous', false);
varfhans = {@linode_dt, @linode_dxdx, @linode_dxdp, @linode_dpdp};
prob1 = ode_isol2forward(prob, '', @linode, @linode_dx,...
    @linode_dp, T0, T, x0, x1, {'k' 'th'}, [0.98; 0.3], varfhans{:});
[data, uidx] = coco_get_func_data(prob1, 'forward', 'data', 'uidx');
bc_funcs = {@linode_bc, @linode_bc_du, @linode_bc_dudu};
prob1 = coco_add_func(prob1, 'po', bc_funcs{:}, [], 'zero', 'uidx', ...
  uidx([data.x0_idx, data.x1_idx, data.T0_idx, data.T_idx]));
prob1 = coco_add_pars(prob1, 'vel', uidx(data.x0_idx(2)), 'v');


% adjoints
prob1 = adjt_isol2forward(prob1, '');
[data, axidx] = coco_get_adjt_data(prob1, 'forward', 'data', 'axidx');
opt = data.forward_opt;
prob1 = coco_add_adjt(prob1, 'po', 'aidx', ...
  axidx([opt.x0_idx, opt.x1_idx, opt.T0_idx, opt.T_idx]));
prob1 = coco_add_adjt(prob1, 'vel', 'd.v', 'aidx', axidx(opt.x0_idx(2)));

% continuation
cont_args = {1, {'v' 'k' 'd.v' 'forward.d.th' 'forward.d.k'}, [0.9 2]};
coco(prob1, 'linode1', [], cont_args{:});

profile viewer

%% Switch at fold to branch with nontrivial multipliers
bd1   = coco_bd_read('linode1');
BPlab = coco_bd_labs(bd1, 'BP');

prob2 = ode_BP2forward(prob, '', 'linode1', BPlab); % Reconstruct 'coll' continuation problem
[data, uidx] = coco_get_func_data(prob2, 'forward', 'data', 'uidx');
prob2 = coco_add_func(prob2, 'po', bc_funcs{:}, [], 'zero', 'uidx', ...
  uidx([data.x0_idx, data.x1_idx, data.T0_idx, data.T_idx]));
prob2 = coco_add_pars(prob2, 'vel', uidx(data.x0_idx(2)), 'v');

% branch switch data
chart = coco_read_solution('linode1', BPlab, 'chart');
cdata = coco_get_chart_data(chart, 'lsol');

% adjoints
prob2 = adjt_BP2forward(prob2, '', 'linode1', BPlab);
[data, axidx] = coco_get_adjt_data(prob2, 'forward', 'data', 'axidx');
opt = data.forward_opt;
[chart, lidx] = coco_read_adjoint('po', 'linode1', BPlab, ...
  'chart', 'lidx');
prob2 = coco_add_adjt(prob2, 'po', 'aidx', ...
  axidx([opt.x0_idx, opt.x1_idx, opt.T0_idx, opt.T_idx]), ...
  'l0', chart.x, 'tl0', cdata.v(lidx));
[chart, lidx] = coco_read_adjoint('vel', 'linode1', BPlab, ...
  'chart', 'lidx');
prob2 = coco_add_adjt(prob2, 'vel', 'd.v', 'aidx', ...
  axidx(opt.x0_idx(2)), 'l0', chart.x, 'tl0', cdata.v(lidx));


% continuation
cont_args = {1, {'d.v', 'v', 'k' 'forward.d.th'}, {[0 1], [.9 2]}};
coco(prob2, 'linode2', [], cont_args{:});


%% Constrain nontrivial multiplier and release additional continuation parameters
bd2 = coco_bd_read('linode2');
lab = coco_bd_labs(bd2, 'EP');

% zero problem
prob3 = ode_forward2forward(prob, '', 'linode2', lab(2));
[data, uidx] = coco_get_func_data(prob3, 'forward', 'data', 'uidx');
prob3 = coco_add_func(prob3, 'po', bc_funcs{:}, [], 'zero', 'uidx', ...
  uidx([data.x0_idx, data.x1_idx, data.T0_idx, data.T_idx]));
prob3 = coco_add_pars(prob3, 'vel', uidx(data.x0_idx(2)), 'v');


% adjoint
prob3 = adjt_forward2forward(prob3, '', 'linode2', lab(2));
[data, axidx] = coco_get_adjt_data(prob3, 'forward', 'data', 'axidx');
opt = data.forward_opt;
chart = coco_read_adjoint('po', 'linode2', lab(2), 'chart');
prob3 = coco_add_adjt(prob3, 'po', 'aidx', ...
  axidx([opt.x0_idx, opt.x1_idx, opt.T0_idx, opt.T_idx]), 'l0', chart.x);
chart = coco_read_adjoint('vel', 'linode2', lab(2), 'chart');
prob3 = coco_add_adjt(prob3, 'vel', 'd.v', 'aidx', ...
  axidx(opt.x0_idx(2)), 'l0', chart.x);

% events
prob3 = coco_add_event(prob3, 'OPT', 'forward.d.th', 0);

% continuation
cont_args = {1, {'forward.d.th' 'v' 'k' 'th'}, {[], [.9 2]}};
coco(prob3, 'linode3', [], cont_args{:});


