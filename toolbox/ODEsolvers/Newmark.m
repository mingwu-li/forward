function [up,vp,varargout] = Newmark(M,N,Nu,Nv,Np,F,Fp,T0,T,x0,p,opts,varargin)
% NEWMARK This function returns the final state of IVP: 
%      M\ddot{u}+N(u,\dot{u})=F(t,p) [u0,v0]=x0
% and the sensitivity of final state with respect to x0
% with initial condition x(T0)=x and the sensitivity of final state with
% respect to x0, T0, T and p. The numerical integration used here is
% Newmark method.
%
% [U,V,VARARGOUT] = NEWMARK(M,N,NU,NV,Np,F,FP,T0,T,X0,P,OPTS,VARARGIN)
%
% M:    mass matrix
% N:    function handle for nonlinear internal force N(u,v) or N(u,v,p)
% Nu:   function handle for dNdu, i.e., derivative w.r.t. displacement
% Nv:   function handle for dNdv, i.e., derivative w.r.t. velocity
% Np:   function handle for dNdp, i.e., derivative w.r.t. parameters
% F:    function handle for external force F(t,p)
% Fp:   function handle for dFdp
% T0:   initial time
% T:    length of time horizon
% p:    problem parameters
% opts: algorithm parameters for integration
% varargin: ['var'], [outdof]; 'var' indicates sensitivity matrices will be
%       calculated. outdof indicates the time history at these dofs will be
%       saved to traj
%
% (up,vp) is the final state. Based on the info of varargin, the
% sensitivity of final state and the time history of the system is presented
% in varargout.

ItMX   = opts.ItMX;
Nsteps = opts.Nsteps;
RelTol = opts.RelTol;
beta  = (1+opts.alpha)^2/4;
gamma = 1/2+opts.alpha;
dt   = T/Nsteps;
dim  = numel(x0)/2; % number of dofs
qdim = numel(p);    % number of parameters
up   = x0(1:dim);
vp   = x0(dim+1:2*dim);
Npden = false; Fpden = false;
if ~isempty(Np); Npden = true; end
if ~isempty(Fp); Fpden = true; end
if Npden
    N_p = N(up,vp,p); % _p stands for evaluation at the previous step
else
    N_p = N(up,vp);
end
if Fpden
    F_0 = F(T0,p);    % _0 stands for evaluation at the initial time
else
    F_0 = F(T0);
end
ap = M\(F_0-N_p);

var  = false;
traj = false;
if ~isempty(varargin)
    arg1 = varargin{1};
    if ischar(arg1) && strcmp(arg1,'var')
        var = true;
    else
        if isnumeric(arg1)
            traj   = true;
            outdof = arg1;
        end
    end
    if numel(varargin)>1
        arg2 = varargin{2};
        if isnumeric(arg2)
            traj   = true;
            outdof = arg2;
        end
    end
end

if var
    Jup  = [-vp,eye(dim),zeros(dim),zeros(dim,qdim)];
    Jvp  = [-ap,zeros(dim),eye(dim),zeros(dim,qdim)];
    Fpp = zeros(dim,1); Npp = zeros(dim,1);
    if Fpden; Fpp = Fp(T0,p); end
    if Npden
        Npp = Np(up,vp,p); Nup = Nu(up,vp,p); Nvp = Nv(up,vp,p);
    else
        Nup = Nu(up,vp); Nvp = Nv(up,vp);
    end
    f0   = [zeros(dim,1),zeros(dim,2*dim),Fpp-Npp];
    Jap  = M\(f0-Nvp*Jvp-Nup*Jup);
end

if traj
    ndofs  = numel(outdof);
    zt = zeros(ndofs,Nsteps+1);
    Et = zeros(1,Nsteps+1); % output of kinetic energy
    zp = [up;vp];
    zt(:,1) = zp(outdof);
    Et(1)   = vp'*M*vp/2;
end
for i=1:Nsteps
    ti = T0+i*dt;
    uc = up; 
    % (up,vp,ap) is the (disp,vel,accel) at previous step
    % (uc,vc,ac) is the solution at current step (unknown)
    % predictor 
    ustar = up+dt*vp+(0.5-beta)*dt^2*ap;
    vstar = vp+(1-gamma)*dt*ap;
    vc = vstar+gamma/(beta*dt)*(uc-ustar);
    ac = (uc-ustar)/(beta*dt^2);
    % Newton-Raphson iteration to yield convergent results
    if Fpden
        Fc = F(ti,p); Fpc = Fp(ti,p);
    else
        Fc = F(ti); Fpc = zeros(dim,1);
    end
    ik = 1;
    while 1
        if Npden
            Nc = N(uc,vc,p); Nuc = Nu(uc,vc,p); Nvc = Nv(uc,vc,p); 
        else
            Nc = N(uc,vc); Nuc = Nu(uc,vc); Nvc = Nv(uc,vc); 
        end
        r = M*ac+Nc-Fc; % residual
        S = M/(beta*dt^2)+gamma*Nvc/(beta*dt)+Nuc;
        du = -S\r;
        uc = uc+du;
        vc = vstar+gamma/(beta*dt)*(uc-ustar);
        ac = (uc-ustar)/(beta*dt^2);        
        if norm(du)<=RelTol*norm(uc)
            % you may further check AbsTol here
            break;
        end
        assert(ik<ItMX,'Newton-Raphson does not coverge');
        ik = ik+1;
    end
    if var
        % update sensitivity matrix
        if Npden
            Npc = Np(uc,vc,p); Nuc = Nu(uc,vc,p); Nvc = Nv(uc,vc,p); 
        else
            Npc = zeros(dim,1); Nuc = Nu(uc,vc); Nvc = Nv(uc,vc); 
        end
        Justar = Jup+dt*Jvp+(0.5-beta)*dt^2*Jap;
        Jvstar = Jvp+(1-gamma)*dt*Jap;
        S   = M/(beta*dt^2)+gamma*Nvc/(beta*dt)+Nuc;
        Fc  = [zeros(dim,1),zeros(dim,2*dim),Fpc-Npc];
        Juc = S\(Fc+M*Justar/(beta*dt^2)+Nvc*(gamma/(beta*dt)*Justar-Jvstar));
        Jvc = Jvstar+gamma/(beta*dt)*(Juc-Justar);
        Jac = (Juc-Justar)/(beta*dt^2);
    end
    % update results
    up  = uc;
    vp  = vc;
    ap  = ac;
    if var
        Jup = Juc;
        Jvp = Jvc;
        Jap = Jac;
    end
    if traj
        zp = [up;vp];
        zt(:,i+1) = zp(outdof);
        Et(i+1) = vp'*M*vp/2;
    end
end

if var && traj
    varargout{1} = Jup;
    varargout{2} = Jvp;
    varargout{3} = zt;
    varargout{4} = Et;
else
    if var
        varargout{1} = Jup;
        varargout{2} = Jvp;
    end
    if traj
        varargout{1} = zt;
        varargout{2} = Et;
    end
end

end