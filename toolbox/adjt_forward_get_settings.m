function data = adjt_ddaecoll_get_settings(prob, tbid, data)
%COLL_GET_SETTINGS   Read 'coll' toolbox instance settings.
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

defaults.NTST = 10; % Number of mesh intervals
defaults.NCOL = 4;  % Degree of interpolating polynomials
defaults.Dpoints = 'Uniform';
defaults.Dnodes  = 'Gauss';
defaults.Apoints = 'Uniform';
defaults.Anodes  = 'Gauss';
if ~isfield(data, 'ddaecoll')
  data.ddaecoll = [];
end
data.ddaecoll = coco_merge(defaults, coco_merge(data.ddaecoll, ...
  coco_get(prob, tbid))); % Defaults < Stored < User-supplied
NTST = data.ddaecoll.NTST;
assert(numel(NTST)==1 && isnumeric(NTST) && mod(NTST,1)==0, ...
  '%s: input for option ''NTST'' is not an integer', tbid);
NCOL = data.ddaecoll.NCOL;
assert(numel(NCOL)==1 && isnumeric(NCOL) && mod(NCOL,1)==0, ...
  '%s: input for option ''NCOL'' is not an integer', tbid);

end
