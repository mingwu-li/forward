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

if data.order==2
    assert(mod(data.dim,2)==0, 'state is of even dimension for second order system');
    dim = data.dim/2;
    data.u0_idx = 1:dim;
    data.v0_idx = dim+1:2*dim;
    data.u1_idx = 2*dim+1:3*dim;
    data.v1_idx = 3*dim+1:4*dim;
end



% data = coco_func_data(data);

end