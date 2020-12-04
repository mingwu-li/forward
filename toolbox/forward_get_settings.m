function data = forward_get_settings(prob, tbid, data)
%COLL_GET_SETTINGS   Read 'forward' toolbox instance settings.
%
% Merge user-supplied toolbox settings with default values.
%
% DATA = COLL_GET_SETTINGS(PROB, TBID, DATA)
%
% PROB - Continuation problem structure.
% TBID - Toolbox instance identifier.
% DATA - Toolbox data strcture.

% Copyright (C) Frank Schilder, Harry Dankowicz
% $Id: coll_get_settings.m 2839 2015-03-05 17:09:01Z fschild $

defaults.autonomous = true;
defaults.order     = 1;
defaults.ODEsolver = @ode45; % use ode45 by default and define solver options
ode_opts           = odeset('RelTol', 1.0e-6, 'AbsTol', 1.0e-8, 'NormControl', 'on');
ode_opts.ItMX      = 10;  % max number of iteration of Newton iteration
ode_opts.Nsteps    = 500; % number of steps in numerical integration
ode_opts.alpha     = 0;   % parameter used in Newmark integration scheme
ode_opts.rhoinf    = 0.9; % parameter used in Galpha integration scheme
defaults.ode_opts  = ode_opts;

copts = coco_get(prob, tbid);
copts = coco_merge(defaults, copts);

data.ode_opts  = copts.ode_opts;
data.ODEsolver = copts.ODEsolver;
data.order     = copts.order;
data.autonomous = copts.autonomous;

ODEsolver = data.ODEsolver;
flag1 = isequal(ODEsolver,@ode45);
flag2 = isequal(ODEsolver,@ode23);
flag3 = isequal(ODEsolver,@ode113);
flag4 = isequal(ODEsolver,@ode15s);
flag5 = isequal(ODEsolver,@ode23s);
flag6 = isequal(ODEsolver,@ode23t);
flag7 = isequal(ODEsolver,@ode23tb);
flag  = any([flag1,flag2,flag3,flag4,flag5,flag6,flag7]);
if flag
    data.MATLABsolver = true;
else
    data.MATLABsolver = false;
end
if data.order==2
    assert(~data.MATLABsolver,'MATLAB solver does not support 2nd order system');
end
end
