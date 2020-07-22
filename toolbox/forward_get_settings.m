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
defaults.ODEsolver = @ode45; % use ode45 by default and define solver options
defaults.ode_opts  = odeset('RelTol', 1.0e-8, 'AbsTol', 1.0e-10, 'NormControl', 'on');

copts = coco_get(prob, tbid);
copts = coco_merge(defaults, copts);

data.ode_opts  = copts.ode_opts;
data.ODEsolver = copts.ODEsolver;
data.autonomous = copts.autonomous;

end
