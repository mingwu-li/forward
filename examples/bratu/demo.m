
T0 = 0;
T  = 1;
x0 = [0; 0];
x1 = x0;
p0 = 0;

% Initialize continuation problem and settings associated with toolbox
% constructor.

prob = coco_prob();
prob = coco_set(prob, 'cont', 'NPR', 20);
% Initial solution guess is the trivial solution u = 0 for p = 0.
prob = ode_isol2forward(prob, '', @brat, @brat_dx,...
    @brat_dp, T0, T, x0, x1, {'p'}, p0);
[data, uidx] = coco_get_func_data(prob, 'forward', 'data', 'uidx');
bc_funcs = {@brat_bc, @brat_bc_du};
prob = coco_add_func(prob, 'bc', bc_funcs{:}, [], 'zero', 'uidx', ...
  uidx([data.T_idx, data.x0_idx, data.x1_idx, data.p_idx]));

fprintf('\n Run=''%s'': Continue family of constrained trajectory segments.\n', ...
  'shoot1');
coco(prob, 'shoot1', [], 1, {'p'}, [0 4]);

%% Continuation in alternative parameterization with branch point
prob = coco_prob();
prob = coco_set(prob, 'cont', 'BP', true);
% Initial solution guess is the trivial solution u = 0 for p = 0.
prob = ode_isol2forward(prob, '', @brat, @brat_dx,...
    @brat_dp, T0, T, x0, x1, {'p'}, p0);
[data, uidx] = coco_get_func_data(prob, 'forward', 'data', 'uidx');
prob = coco_add_func(prob, 'bc', bc_funcs{:}, [], 'zero', 'uidx', ...
  uidx([data.T_idx, data.x0_idx, data.x1_idx, data.p_idx]));
prob = coco_add_func(prob, 'trans', @trans, [], 'zero', 'uidx', ...
  uidx(data.p_idx), 'u0', 0);
uidx = coco_get_func_data(prob, 'trans', 'uidx');
prob = coco_add_pars(prob, 'pars', uidx(end), 'C');

fprintf('\n Run=''%s'': Continue primary family of constrained trajectory segments.\n', ...
  'shoot2');

bd2 = coco(prob, 'shoot2', [], 1, {'C' 'p'}, [0 5]);

BPlab = coco_bd_labs(bd2, 'BP');
prob = coco_prob();
prob = coco_set(prob, 'cont', 'BP', true);
% Initial solution guess is the trivial solution u = 0 for p = 0.
prob = ode_BP2forward(prob, '', 'shoot2', BPlab);
[data, uidx] = coco_get_func_data(prob, 'forward', 'data', 'uidx');
prob = coco_add_func(prob, 'bc', bc_funcs{:}, [], 'zero', 'uidx', ...
  uidx([data.T_idx, data.x0_idx, data.x1_idx, data.p_idx]));
chart = coco_read_solution('', 'shoot2', BPlab, 'chart');
cdata = coco_get_chart_data(chart, 'lsol');
pidx  = uidx(data.p_idx);
prob  = coco_add_func(prob, 'trans', @trans, [], 'zero', 'uidx', ...
  uidx(data.p_idx), 'u0', 2.3994, 't0', cdata.v(pidx+1));
uidx  = coco_get_func_data(prob, 'trans', 'uidx');
prob  = coco_add_pars(prob, 'pars', uidx(end), 'C');

fprintf(...
  '\n Run=''%s'': Continue secondary family of constrained trajectory segments from point %d in run ''%s''.\n', ...
  'shoot3', BPlab, 'shoot2');

bd3 = coco(prob, 'shoot3', [], 1, {'C' 'p'}, [0 5]);
