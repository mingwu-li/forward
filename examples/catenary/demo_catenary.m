coco_use_recipes_toolbox forward_v2 % add the coll_v1 toolbox to the search path

%% Section 7.3.1

% The continuation problem encoded below corresponds to a family of 98 zero
% function (80 collocation conditions, 18 continuity conditions), in terms
% of 101 continuation variables (100 basepoint values and one interval
% length), a family of four monitor functions that evaluate to the
% components of the trajectory end point at t=0, the first component of the
% trajectory end point at t=1, and the interval length, respectively, and
% four corresponding inactive continuation parameters 'y1s', 'y2s', 'y1e',
% and 'T'. Its dimensional deficit equals -1. The call to the coco
% entry-point function indicates a desired manifold dimension of 1. To this
% end, the continuation parameter 'T' and 'y1' are released and allowed to
% vary during continuation.

t0 = [0; 0.04];
T0 = 0;
T  = 1;
x0 = [1 0]';
% forward simulation with ode45
f = @(t,x) catenary(x, []);
[t,x] = ode45(f, [T0, T0+T], x0);
x1 = x(end,:)';
prob = coco_prob();
prob = ode_isol2forward(prob, '', @catenary, @catenary_dx, @catenary_dp, ...
    T0, T, x0, x1, {}, []);

data = coco_get_func_data(prob, 'forward', 'data'); % Extract toolbox data
prob = coco_add_pars(prob, 'pars', ...
  [data.x0_idx data.x1_idx data.T_idx], ...
  {'y1s' 'y2s' 'y1e' 'y2e' 'T'});
coco(prob, 'shoot1', [], 1, {'T' 'y1e' 'y2e'}, [0.1 3]);

%%
prob = coco_prob();
prob = ode_forward2forward(prob, '', 'shoot1', 7); % Reconstruct 'coll' continuation problem
data = coco_get_func_data(prob, 'forward', 'data'); % Extract toolbox data
prob = coco_add_pars(prob, 'pars', ...
  [data.x0_idx data.x1_idx data.T_idx], ...
  {'y1s' 'y2s' 'y1e' 'y2e' 'T'});
coco(prob, 'shoot2', [], 1, {'y2e' 'y2s' 'y1e'}, [9 12]);


%%
prob = coco_prob();
prob = ode_forward2forward(prob, '', 'shoot1', 7); % Reconstruct 'coll' continuation problem
data = coco_get_func_data(prob, 'forward', 'data'); % Extract toolbox data
prob = coco_add_pars(prob, 'pars', ...
  [data.x0_idx data.x1_idx data.T_idx], ...
  {'y1s' 'y2s' 'y1e' 'y2e' 'T'});
coco(prob, 'shoot3', [], 1, {'y1e' 'y2s' 'y2e'}, [9 12]);


coco_use_recipes_toolbox % remove the coll_v1 toolbox from the search path
