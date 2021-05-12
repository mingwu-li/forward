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

par_opts.parallel = false;    % integration in parallel computation
par_opts.ncores   = 1;        % number of cores in parallel computation
par_opts.subdim   = 0;       % dimension of state for each sub-problem
defaults.par_opts = par_opts;

copts = coco_get(prob, tbid);
copts = coco_merge(defaults, copts);

if copts.par_opts.parallel
    % check subdim
    assert(copts.par_opts.subdim>0, 'the state dimension of subproblem should be larger than zero');
    % activate parallel computing
    activate_parallel(copts.par_opts.ncores)
end

data.ode_opts  = copts.ode_opts;
data.par_opts  = copts.par_opts;
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

function activate_parallel(varargin)
%ACTIVATE_PARALLEL This function starts parallel pool for the first time
%and detects the number of cores avaliable

try    
    pp1 = gcp('nocreate');
    if isempty(pp1)
        h = helpdlg('Starting parallel pool for the first time and detecting number of available cores.', 'Info');
        disp('----------------------------------------------------------------')
        disp('Starting parallel pool for the first time and detecting number')
        disp('of available cores.')
        disp('----------------------------------------------------------------')
        defaultProfile = parallel.defaultClusterProfile;
        myCluster = parcluster(defaultProfile);
        if numel(varargin)>=1 && isnumeric(varargin{1})
            parpool(myCluster,varargin{1});
        else
            parpool(myCluster);
        end
        pp2 = gcp('nocreate');
        cpuNum =pp2.NumWorkers;
        save('cluster_info.mat','cpuNum')

        if isvalid(h)
            close(h)
        end
    end
catch ME
    rethrow(ME)
end

end
