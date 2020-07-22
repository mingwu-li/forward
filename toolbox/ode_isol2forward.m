function prob = ode_isol2forward(prob, oid, f, fx, fp, T0, T, x0, x1, pnames, p0, varargin)
%ODE_ISOL2FORWARD   Append 'forward' instance constructed from initial data.
%
% Parse input sequence to construct toolbox data and initial solution guess
% and use this to construct an instance of 'forward'.
%
% PROB     = ODE_ISOL2FORWARD(PROB, OID, @F @DFDX @DFDP T0 T X0 X1 PNAMES P0)
% VARARGIN = { @DFDT|[], @DFDXDX|[], @DFDXDP|[], @DFDPDP|[] }
%
% PROB   - Continuation problem structure.
% OID    - Object instance identifier (string).
% @F     - Function handle to vector field.
% @DFDX  - Optional function handle to Jacobian w.r.t. problem variables.
% @DFDP  - Optional function handle to Jacobian w.r.t. problem parameters.
% T0     - Initial time.
% T      - Time period.
% X0     - Initial state.
% X1     - Final state.
% PNAMES - Optional string label or cell array of string labels for
%          continuation parameters tracking problem parameters.
% P0     - Initial solution guess for problem parameters.

tbid = coco_get_id(oid, 'forward'); % Create toolbox instance identifier

fields = {'tbid' 'f' 'fx' 'fp'
  tbid   f   fx   fp};
data = struct(fields{:});
data.pnames = pnames;

data.ft  = [];
data.fxx = [];
data.fxp = [];
data.fpp = [];

argidx = 1;
if argidx<=nargin-11
    data.ft = varargin{argidx};
    argidx  = argidx+1;
    if argidx<=nargin-11
        data.fxx = varargin{argidx};
        argidx   = argidx+1;
        if argidx<=nargin-11
            data.fxp = varargin{argidx};
            argidx   = argidx+1;
            if argidx<=nargin-11
                data.fpp = varargin{argidx};
            end
        end
    end
end

t0   = [];
forward_arg_check(tbid, data, T0, T, x0, x1, p0);    % Validate input
data = forward_get_settings(prob, tbid, data);       % Get toolbox settings
data = forward_init_data(data, x0, p0);              % Build toolbox data
sol  = forward_init_sol(T0, T, x0, x1, p0, t0);      % Build initial solution guess
prob = ode_construct_forward(prob, tbid, data, sol); % Append continuation problem

end
