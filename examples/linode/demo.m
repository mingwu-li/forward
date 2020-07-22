
T0 = 0;
T  = 2*pi;
x0 = [0.276303; 0.960863];
p0 = [0.98; 0.3];

% forward simulation with ode45
f = @(t,x) linode(t, x, p0);
[t,x] = ode45(f, [T0, T0+T], x0);
x1 = x0;
prob = coco_prob();
prob = coco_set(prob, 'forward', 'autonomous', false);
prob = ode_isol2forward(prob, '', @linode, @linode_dx,...
    @linode_dp, T0, T, x0, x1, {'k' 'th'}, [0.98; 0.3]);

[data, uidx] = coco_get_func_data(prob, 'forward', 'data', 'uidx');
bc_funcs = {@linode_bc, @linode_bc_du, @linode_bc_dudu};
prob = coco_add_func(prob, 'po', bc_funcs{:}, [], 'zero', 'uidx', ...
  uidx([data.x0_idx, data.x1_idx, data.T0_idx, data.T_idx]));
prob = coco_add_pars(prob, 'vel', uidx(data.x0_idx(2)), 'v');
coco(prob, 'shoot1', [], 1, {'v' 'k'}, [0.9 2]);

%%
prob = coco_prob();
prob = coco_set(prob, 'forward', 'autonomous', false);
prob = ode_forward2forward(prob, '', 'shoot1', 4); % Reconstruct 'coll' continuation problem
[data, uidx] = coco_get_func_data(prob, 'forward', 'data', 'uidx');
prob = coco_add_func(prob, 'po', bc_funcs{:}, [], 'zero', 'uidx', ...
  uidx([data.x0_idx, data.x1_idx, data.T0_idx, data.T_idx]));
prob = coco_add_pars(prob, 'vel', uidx(data.x0_idx(2)), 'v');
coco(prob, 'shoot2', [], 1, {'v' 'th'}, [0.7 2]);




