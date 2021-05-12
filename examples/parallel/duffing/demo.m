% We consider a composite problem with two uncoupled duffing oscillators
% subject to harmonic excitation for the purpose of testing the
% functionality of parallel numerical integration

%% sequential
%% initial solution guess
p0 = [0.06; 1];
T0 = 0;
T  = 2*pi/p0(2);

% forward simulation with ode45
f = @(t,x) duffing(t, x, p0);
options = odeset('RelTol',1e-8,'AbsTol',1e-10);
[~,x] = ode45(f, 0:100*T, [0;0], options);
[tt,x] = ode45(f, [0 T], x(end,:), options);
x0 = x(1,:);
x1 = x(end,:);

prob = coco_prob();
prob = coco_set(prob, 'forward', 'autonomous', false);
%% first run
% zero problems
forward_args = {@duffing_seq, @duffing_seq_dx, @duffing_seq_dp, T0, T, [x0 x0], [x1 x1], {'amp' 'omega'}, p0};
prob1 = ode_isol2forward(prob, '',forward_args{:});

[data, uidx] = coco_get_func_data(prob1, 'forward', 'data', 'uidx');
bc_funcs = {@duffing_bc};
prob1 = coco_add_func(prob1, 'bc', bc_funcs{:}, [], 'zero', 'uidx', ...
  uidx([data.x0_idx, data.x1_idx, data.T0_idx, data.T_idx, data.p_idx(2)]));


% continuation
cont_pars = {'amp' 'omega'};
coco(prob1, 'duffing1', [], 1, cont_pars, [0.06 0.5]);

%% second run
prob1 = ode_forward2forward(prob, '', 'duffing1',3);
[data, uidx] = coco_get_func_data(prob1, 'forward', 'data', 'uidx');
bc_funcs = {@duffing_bc};
prob1 = coco_add_func(prob1, 'bc', bc_funcs{:}, [], 'zero', 'uidx', ...
  uidx([data.x0_idx, data.x1_idx, data.T0_idx, data.T_idx, data.p_idx(2)]));


% continuation
cont_pars = {'omega' 'amp'};
coco(prob1, 'duffing1_omega', [], 1, cont_pars, [0.5 1.5]);


%% parallel

prob = coco_prob();
par_opts.parallel = true;
par_opts.subdim = 2;
par_opts.ncores = 2;
prob = coco_set(prob, 'forward', 'par_opts', par_opts);
prob = coco_set(prob, 'forward', 'autonomous', false);
%% first run
% zero problems
forward_args = {@duffing, @duffing_dx, @duffing_dp, T0, T, [x0 x0], [x1 x1], {'amp' 'omega'}, p0};
prob1 = ode_isol2forward(prob, '',forward_args{:});

[data, uidx] = coco_get_func_data(prob1, 'forward', 'data', 'uidx');
bc_funcs = {@duffing_bc};
prob1 = coco_add_func(prob1, 'bc', bc_funcs{:}, [], 'zero', 'uidx', ...
  uidx([data.x0_idx, data.x1_idx, data.T0_idx, data.T_idx, data.p_idx(2)]));

% continuation
cont_pars = {'amp' 'omega'};
coco(prob1, 'duffing2', [], 1, cont_pars, [0.06 0.5]);

%% second run
prob1 = ode_forward2forward(prob, '', 'duffing2',3);
[data, uidx] = coco_get_func_data(prob1, 'forward', 'data', 'uidx');
bc_funcs = {@duffing_bc};
prob1 = coco_add_func(prob1, 'bc', bc_funcs{:}, [], 'zero', 'uidx', ...
  uidx([data.x0_idx, data.x1_idx, data.T0_idx, data.T_idx, data.p_idx(2)]));


% continuation
cont_pars = {'omega' 'amp'};
coco(prob1, 'duffing2_omega', [], 1, cont_pars, [0.5 1.5]);

deactivate_parallel()