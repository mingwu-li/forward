function sol = forward_read_adjoint(oid, run, lab, varargin)
%COLL_READ_ADJOINT   Read adjoint data from disk.
%
% [SOL DATA] = FORWARD_READ_ADJOINT(OID, RUN, LAB, VARARGIN)
%
% VARARGIN = { [BRANCH_TYPE] }
% Read adjoint data from solution data file of run RUN with solution label
% LAB.
%
% Construct initial adjoint solution and data structure.
%
% SOL  - Solution (struct).
% OID  - Object instance identifier (string).
% RUN  - Run identifier (string).
% LAB  - Solution label (integer).
% BRANCH_TYPE - Branch type ('BP').
%
% The solution structure SOL will have the fields
%
%   SOL.L       :  Lagrange multipliers for x(end,:)-x1.
%   SOL.TL      :  Differentials of Lagrange multipliers for trajectory
%                  segment.
%   SOL.PARS_L  :  Lagrange multiplier for parameters.
%   SOL.PARS_TL :  Differential of Lagrange multiplier for parameters.
%   SOL.T0_L    :  Lagrange multiplier for initial time.
%   SOL.T0_TL   :  Differential of Lagrange multiplier for initial time.
%
% See also: COCO_READ_ADJOINT, ADJT_ISOL2FORWARD, ADJT_FORWARD2FORWARD,
% FORWARD_ADJT_INIT_DATA


if isempty(oid) && isempty(run)
    sol.l0       = [];
    sol.tl0      = [];
    sol.pars_l0  = [];
    sol.pars_tl0 = [];
    sol.T0_l0    = [];
    sol.T0_tl0   = [];
    return
end

tbid = coco_get_id(oid, 'forward');

if ~isempty(varargin) && strcmp(varargin{1},'BP')
    [data, chart, lidx] = coco_read_adjoint(tbid, run, lab, 'data', ...
        'chart', 'lidx');
    sol.l0 = chart.x;
    cdata  = coco_get_chart_data(chart, 'lsol');
    if isempty(cdata)
      coco_warn([], 1, 1, ...
        'Could not find restart data for branch-switching\n');
    else
      sol.tl0 = cdata.v(lidx);
    end
    [chart, lidx] = coco_read_adjoint(coco_get_id(tbid, 'pars'), run, ...
        lab, 'chart', 'lidx');
    sol.pars_l0  = chart.x;
    sol.pars_tl0 = cdata.v(lidx);
    if data.forward_seg.autonomous
        [chart, lidx] = coco_read_adjoint(coco_get_id(tbid, 'T0'), ...
            run, lab, 'chart', 'lidx');
        sol.T0_l0  = chart.x;
        sol.T0_tl0 = cdata.v(lidx);
    end    
else
    [data, chart] = coco_read_adjoint(tbid, run, lab, 'data', 'chart');
    sol.l0  = chart.x;
    sol.tl0 = [];
    chart = coco_read_adjoint(coco_get_id(tbid, 'pars'), run, lab, 'chart');
    sol.pars_l0  = chart.x;
    sol.pars_tl0 = [];
    if data.forward_seg.autonomous
        chart = coco_read_adjoint(coco_get_id(tbid, 'forward.T0'), run, ...
            lab, 'chart');
        sol.T0_l0  = chart.x;
        sol.T0_t10 = [];
    end
end 

end