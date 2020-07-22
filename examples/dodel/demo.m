

T0 = 0;
T  = 1;
x0 = [0.2; 0.5];
p0 = [1; 1];
% forward simulation with ode45
f = @(t,x) doedel(x, p0);
[t,x] = ode45(f, [T0, T0+T], x0);
x1 = x(end,:)';
prob = coco_prob();
prob = ode_isol2forward(prob, '', @doedel, @doedel_DFDX, @doedel_DFDP, ...
    T0, T, x0, x1, {'p1','p2'}, p0);

data = coco_get_func_data(prob, 'forward', 'data'); % Extract toolbox data
prob = coco_add_pars(prob, 'pars', ...
  [data.x0_idx data.x1_idx data.T_idx], ...
  {'y1s' 'y2s' 'y1e' 'y2e' 'T'});
coco(prob, 'shoot1', [], 1, {'T' 'y1e' 'y2e'}, [0.1 2]);

%%
prob = coco_prob();
prob = ode_forward2forward(prob, '', 'shoot1', 5); % Reconstruct 'coll' continuation problem
data = coco_get_func_data(prob, 'forward', 'data'); % Extract toolbox data
prob = coco_add_pars(prob, 'pars', ...
  [data.x0_idx data.x1_idx data.T_idx], ...
  {'y1s' 'y2s' 'y1e' 'y2e' 'T'});
coco(prob, 'shoot3', [], 1, {'y2e' 'y2s' 'y1e'}, [5 10]);




