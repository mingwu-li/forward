function install
cocodir = fileparts(mfilename('fullpath'));
addpath(fullfile(cocodir, 'toolbox'));
addpath(fullfile(cocodir, 'toolbox', 'ODEsolvers'));
addpath(fullfile(cocodir, 'toolbox', 'myChebfun'));
end

% function startup
% cocodir = fileparts(fileparts(mfilename('fullpath')));
% addpath(fullfile(cocodir, 'toolbox'));
% cp = coco_path;
% addpath(cp{:});
% end
