function data = forward_adjt_init_data(prob, src_data) %#ok<INUSL>
%COLL_ADJT_INIT_DATA   Initialize data structure for a 'coll' adjoint problem.
%
% See also FORWARD_INIT_DATA, COCO_SAVE_DATA, COCO_FUNC_DATA.

data.forward_seg = src_data;
dim  = src_data.dim;
pdim = src_data.pdim;

opt.adim    = [dim, 2*dim+pdim+2];
opt.x0_idx  = 1:dim;
opt.x1_idx  = opt.x0_idx(end) + (1:dim);
opt.T0_idx  = opt.x1_idx(end) + 1;
opt.T_idx   = opt.T0_idx + 1;
opt.p_idx   = opt.T_idx + (1:pdim);

var2      = struct();
var2.phi  = 1:dim;
var2.x0   = var2.phi(end)+1:dim+dim*dim;
var2.T0   = var2.x0(end)+1:2*dim+dim*dim;
var2.p    = var2.T0(end)+1:2*dim+dim*dim+dim*pdim;
var2.T0T0 = var2.p(end)+1:3*dim+dim*dim+dim*pdim;
var2.T0p  = var2.T0T0(end)+1:3*dim+dim*dim+2*dim*pdim;
var2.x0T0 = var2.T0p(end)+1:3*dim+2*dim*dim+2*dim*pdim;
var2.x0x0 = var2.x0T0(end)+1:3*dim+2*dim*dim+2*dim*pdim+dim*dim*dim;
var2.x0p  = var2.x0x0(end)+1:3*dim+2*dim*dim+2*dim*pdim+dim*dim*(dim+pdim);
var2.pp   = var2.x0p(end)+1:3*dim+2*dim*dim+2*dim*pdim+dim*dim*(dim+pdim)+dim*pdim*pdim;
opt.var2  = var2;

data.forward_opt = opt;

% data = coco_func_data(data);

end
