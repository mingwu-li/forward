function forward_arg_check(tbid, data, T0, T, x0, x1, p0)
%COLL_ARG_CHECK   Basic argument checking for 'forward' toolbox.
%
% Validate user-supplied inputs and terminate execution with suitable error
% messages if the inputs fail to be of the correct type.
%
% FORWARD_ARG_CHECK(TBID, DATA, T0, X0, X1, P0)
%
% TBID - Toolbox instance identifier.
% DATA - Toolbox data structure.
% T0   - Initial time.
% T    - Time period
% X0   - Initial state.
% X1   - Final state
% P0   - Initial solution guess for problem parameters.

assert(numel(T0)==1, ...
  '%s: T0 is not a sclar', ...
  tbid);
assert(numel(T)==1, ...
  '%s: T is not a sclar', ...
  tbid);
assert(numel(x0)==numel(x1), ...
  '%s: incompatible number of elements for ''x0'' and ''x1''', ...
  tbid);
assert(numel(p0)==numel(data.pnames) || isempty(data.pnames), ...
  '%s: incompatible number of elements for ''p0'' and ''pnames''', ...
  tbid);

end
