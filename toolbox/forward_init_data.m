function data = forward_init_data(data, x0, p0)
%COLL_INIT_DATA   Initialize toolbox data for an instance of 'forward'.
%
% Populate remaining fields of the toolbox data structure used by 'forward'
% function objects.
%
% DATA = FORWARD_INIT_DATA(DATA, X0, P0)
%
% DATA - Toolbox data structure.
% X0   - Initial solution guess initial states.
% P0   - Initial solution guess for problem parameters.

data.dim    = numel(x0);    % Dimension of state
data.pdim   = numel(p0);    % Number of problem parameters
data.x0_idx = 1:data.dim;
data.x1_idx = data.x0_idx(end) + (1:data.dim);
data.T0_idx = data.x1_idx(end) + 1;
data.T_idx  = data.T0_idx + 1;
data.p_idx  = data.T_idx + (1:data.pdim);

% data = coco_func_data(data);

end