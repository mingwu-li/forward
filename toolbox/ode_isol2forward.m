function prob = ode_isol2forward(prob, oid, varargin)
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

data = struct();
data = forward_get_settings(prob, tbid, data);       % Get toolbox settings
switch data.order
    case 1
        grammar   = 'F DFDX DFDP [DFDT [DFDXDX [DFDXDP [DFDPDP]]]] T0 T X0 X1 [PNAMES] P0';
        args_spec = {
               'F',     '',     '@',      'fhan', [], 'read', {}
            'DFDX',     '',     '@',   'dfdxhan', [], 'read', {}
            'DFDP',     '',     '@',   'dfdphan', [], 'read', {}
            'DFDT',     '',  '@|[]',   'dfdthan', [], 'read', {}
          'DFDXDX',     '',  '@|[]', 'dfdxdxhan', [], 'read', {}
          'DFDXDP',     '',  '@|[]', 'dfdxdphan', [], 'read', {}
          'DFDPDP',     '',  '@|[]', 'dfdpdphan', [], 'read', {}
              'T0',     '', '[num]',        'T0', [], 'read', {}
               'T',     '', '[num]',         'T', [], 'read', {}
              'X0',     '', '[num]',        'x0', [], 'read', {}
              'X1',     '', '[num]',        'x1', [], 'read', {}
          'PNAMES', 'cell', '{str}',    'pnames', {}, 'read', {}
              'P0',     '', '[num]',        'p0', [], 'read', {}
          };
        [args, ~] = coco_parse(grammar, args_spec, [], varargin{:});
        data.f   = args.fhan;
        data.fx  = args.dfdxhan;
        data.fp  = args.dfdphan;
        data.ft  = args.dfdthan;
        data.fxx = args.dfdxdxhan;
        data.fxp = args.dfdxdphan;
        data.fpp = args.dfdpdphan;
        
    case 2
        grammar   = 'M N DNDU DNDV DNDP F [DFDP] T0 T X0 X1 [PNAMES] P0';
        args_spec = {
               'M',     '', '[num]',         'M', [], 'read', {}
               'N',     '',     '@',      'Nhan', [], 'read', {}
            'DNDU',     '',  '@|[]',   'dNduhan', [], 'read', {}
            'DNDV',     '',  '@|[]',   'dNdvhan', [], 'read', {}
            'DNDP',     '',  '@|[]',   'dNdphan', [], 'read', {}
               'F',     '',  '@|[]',      'Fhan', [], 'read', {}
            'DFDP',     '',  '@|[]',   'dFdphan', [], 'read', {}
              'T0',     '', '[num]',        'T0', [], 'read', {}
               'T',     '', '[num]',         'T', [], 'read', {}
              'X0',     '', '[num]',        'x0', [], 'read', {}
              'X1',     '', '[num]',        'x1', [], 'read', {}
          'PNAMES', 'cell', '{str}',    'pnames', {}, 'read', {}
              'P0',     '', '[num]',        'p0', [], 'read', {}
          };
        [args, ~] = coco_parse(grammar, args_spec, [], varargin{:});   
        data.M  = args.M;
        data.N  = args.Nhan;
        data.Nu = args.dNduhan;
        data.Nv = args.dNdvhan;
        data.Np = args.dNdphan;
        data.F  = args.Fhan;
        data.Fp = args.dFdphan;
            
    otherwise
        error('The order of ODE should be either 1 or 2.');
end

T0 = args.T0;
T  = args.T;
x0 = args.x0;
x1 = args.x1;
p0 = args.p0;

data.pnames = args.pnames;

t0   = [];
forward_arg_check(tbid, data, T0, T, x0, x1, p0);    % Validate input
data = forward_init_data(data, x0, p0);              % Build toolbox data
sol  = forward_init_sol(T0, T, x0, x1, p0, t0);      % Build initial solution guess
prob = ode_construct_forward(prob, tbid, data, sol); % Append continuation problem

end
