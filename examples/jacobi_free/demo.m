% a brief demo

prob = coco_prob();
% prob = coco_set(prob, 'cont', 'corrector', 'broydengood'); % one-time
% computation of jacobian matrix at each continuation step
prob = coco_set(prob, 'cont', 'corrector', 'fsolve');
x0 = zeros(2,1); om = 0.6;
u0 = [x0; om];
prob = coco_add_func(prob, 'forward_map', @forward_map,  [], 'zero', 'u0', u0);
uidx = coco_get_func_data(prob, 'forward_map', 'uidx');
prob = coco_add_pars(prob, 'om', uidx(end), 'om'); % the last index corresponds to omega
coco(prob, 'run1', [], 1, {'om'}, [0.1 1.6]);

% post-processing
sol = coco_read_solution('forward_map','run1',6,'chart');
xinit  = sol.x(1:2);
omsamp = sol.x(3);
odefun = @(t,x) duffing(t,x,omsamp);
[ts,xt] = ode45(odefun,[0,2*pi/omsamp],xinit);
figure; plot(ts,xt);
figure; plot(xt(:,1),xt(:,2))