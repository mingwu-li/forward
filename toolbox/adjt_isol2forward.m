function prob = adjt_isol2forward(prob, oid)
%ADJT_ISOL2FORWARD   Append adjoint of 'forward' instance from initial guess.
%
% PROB = ADJT_ISOL2FORWARD(PROB, OID)
%
% Append adjoint of a 'forward' instance with object instance identifier OID
% that has been previously added to the continuation problem contained in
% PROB using ODE_ISOL2FORWARD. The preceding call to ODE_ISOL2FORWARD must
% include explicit Jacobians, while functions evaluating second derivatives
% are optional. 
%
% On input:
%
% PROB   : Continuation problem structure.
%
% OID    : Object instance identifier (string). The corresponding toolbox
%          instance identifier is coco_get_id(OID, 'forward'). Pass the empty
%          string '' for a simple continuation of trajectory segments. Pass
%          a non-trivial object identifier if an instance of the FORWARD
%          toolbox is part of a composite continuation problem.
%
% See also: ODE_ISOL2FORWARD, FORWARD_READ_ADJOINT, FORWARD_ADJT_INIT_DATA,
% COLL_CONSTRUCT_ADJT

tbid = coco_get_id(oid, 'forward');
data = coco_get_func_data(prob, tbid, 'data');

data = forward_adjt_init_data(prob, data);
sol  = forward_read_adjoint('', '', data);
prob = forward_construct_adjt(prob, tbid, data, sol);

end
