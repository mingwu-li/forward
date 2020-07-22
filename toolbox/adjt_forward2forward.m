function prob = adjt_forward2forward(prob, oid, varargin)
%ADJT_FORWARD2FORWARD   Append adjoint of 'forward' instance from saved solution.
%
% PROB = ADJT_FORWARD2FORWARD(PROB, OID, VARARGIN)
% VARARGIN = { RUN [SOID] LAB }
%
% Append adjoint of a 'coll' instance with object instance identifier OID
% that has been previously added to the continuation problem contained in
% PROB from the same saved solution using ODE_COLL2COLL.
%
% PROB     = ODE_FORWARD2FORWARD(PROB, OID, VARARGIN)
% VARARGIN = { RUN [SOID] LAB }
%
% PROB - Continuation problem structure.
% OID  - Target object instance identifier (string).
% RUN  - Run identifier (string).
% SOID - Source object instance identifier (string).
% LAB  - Solution label (integer).

tbid = coco_get_id(oid, 'forward'); % Create toolbox instance identifier
data = coco_get_func_data(prob, tbid, 'data');
str  = coco_stream(varargin{:}); % Convert varargin to stream of tokens for argument parsing
run  = str.get;
if ischar(str.peek)
  soid = str.get;
else
  soid = oid;
end
lab = str.get;

data  = forward_adjt_init_data(prob, data);  % Build toolbox data
sol   = forward_read_adjoint(soid, run, lab);  % Extract solution and toolbox data from disk
prob  = forward_construct_adjt(prob, tbid, data, sol); % Append continuation problem

end
