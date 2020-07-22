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

assert(isa(data.f, 'function_handle'), ...
  '%s: input for ''f'' is not a function handle', tbid);
assert(isa(data.fx, 'function_handle'), ...
  '%s: input for ''fx'' is not a function handle', tbid);
assert(isa(data.fp, 'function_handle'), ...
  '%s: input for ''fp'' is not a function handle', tbid);
assert(isnumeric(T0), '%s: input for ''T0'' is not numeric', tbid);
assert(isnumeric(T),  '%s: input for ''T''  is not numeric', tbid);
assert(isnumeric(x0), '%s: input for ''x0'' is not numeric', tbid);
assert(isnumeric(x1), '%s: input for ''x1'' is not numeric', tbid);
assert(numel(x0)==numel(x1), ...
  '%s: incompatible number of elements for ''x0'' and ''x1''', ...
  tbid);
assert(isnumeric(p0), '%s: input for ''p0'' is not numeric', tbid);
assert(numel(p0)==numel(data.pnames) || isempty(data.pnames), ...
  '%s: incompatible number of elements for ''p0'' and ''pnames''', ...
  tbid);
assert(is_empty_or_func(data.ft), ...
    '%s: input for ''ft'' is neither empty nor a function handle', tbid);
assert(is_empty_or_func(data.fxx), ...
    '%s: input for ''fxx'' is neither empty nor a function handle', tbid);
assert(is_empty_or_func(data.fxp), ...
    '%s: input for ''fxp'' is neither empty nor a function handle', tbid);
assert(is_empty_or_func(data.fpp), ...
    '%s: input for ''fpp'' is neither empty nor a function handle', tbid);
end


function flag = is_empty_or_func(x)
%IS_EMPTY_OR_FUNC   Check if input is empty or contains a function handle.
flag = isempty(x) || isa(x, 'function_handle');
end
