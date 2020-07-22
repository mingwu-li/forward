%% Stationary points in Duffing oscillator
%
% We apply the method of staged continuation to the search for stationary
% values of amp along a family of periodic solutions of the harmonically
% excited Duffing oscillator
%
%     x'' + 2*zeta*x' + x + alpha*x^3 = amp*cos(omega*t)
% x1 = x; x2 = x'; x3 = omega*t
%
% In the first stage of continuation, a local extremum in amp is detected
% as a fold along the solution manifold. By construction, this coincides
% with a branch point of the augmented continuation problem that includes
% the adjoint conditions associated with the zero and monitor functions. In
% the second stage, we continue along the secondary branch until the
% Lagrange multiplier associated with amp equals 1. As described in the
% tutorial document, at this point, 'amp' = 2*'alpha'*'d.alpha'

%% initial solution guess
p0 = [0.1; 0.5; 0.06; 1.2];
x0 = [p0(3)/(1-p0(4)^2); 0; 0];
T0 = 0;
T  = 2*pi/p0(4);
x1 = x0;
x1(3) = 2*pi;

prob = coco_prob();
prob = coco_set(prob, 'cont', 'BP', true);
prob = coco_set(prob, 'cont', 'FP', true);
%% first run to find initial fold
% zero problems
forward_args = {@duffing, @duffing_dx, @duffing_dp, T0, T, x0, x1, {'zeta' 'alpha' 'amp' 'omega'}, p0};
prob1 = ode_isol2forward(prob, '',forward_args{:});

[data, uidx] = coco_get_func_data(prob1, 'forward', 'data', 'uidx');
bc_funcs = {@duffing_bc, @duffing_bc_du, @duffing_bc_dudu};
prob1 = coco_add_func(prob1, 'bc', bc_funcs{:}, [], 'zero', 'uidx', ...
  uidx([data.x0_idx, data.x1_idx]));

% adjoints
prob1 = adjt_isol2forward(prob1, '');
[data, axidx] = coco_get_adjt_data(prob1, 'forward', 'data', 'axidx');
opt = data.forward_opt;
prob1 = coco_add_adjt(prob1, 'bc', 'aidx', axidx([opt.x0_idx, opt.x1_idx]));

% continuation
cont_pars = {'amp' 'forward.d.amp' 'forward.d.zeta' 'forward.d.alpha' 'forward.d.omega' 'd.forward.T0'};
coco(prob1, 'duffing1', [], 1, cont_pars, [0.06 0.5]);
 
%% switch at fold to branch with nontrivial multipliers
bd1 = coco_bd_read('duffing1');
BPlab = coco_bd_labs(bd1, 'BP');

% zero problems
prob2 = ode_BP2forward(prob, '', 'duffing1', BPlab(1));   
[data, uidx] = coco_get_func_data(prob2, 'forward', 'data', 'uidx');
prob2 = coco_add_func(prob2, 'bc', bc_funcs{:}, [], 'zero', 'uidx', ...
  uidx([data.x0_idx, data.x1_idx]));

% branch switch data
chart = coco_read_solution('duffing1', BPlab(1), 'chart');
cdata = coco_get_chart_data(chart, 'lsol');

% adjoints
prob2 = adjt_BP2forward(prob2, '', 'duffing1', BPlab(1));
[data, axidx] = coco_get_adjt_data(prob2, 'forward', 'data', 'axidx');
opt = data.forward_opt;
[chart, lidx] = coco_read_adjoint('bc', 'duffing1', BPlab(1), ...
  'chart', 'lidx');
prob2 = coco_add_adjt(prob2, 'bc', 'aidx', ...
  axidx([opt.x0_idx, opt.x1_idx]), 'l0', chart.x, 'tl0', cdata.v(lidx));

% continuation
cont_pars = {'forward.d.amp' 'amp' 'forward.d.zeta' 'forward.d.alpha' 'forward.d.omega' 'd.forward.T0'};
coco(prob2, 'duffing2', [], 1, cont_pars, {[0 1] [0.1 2]});
