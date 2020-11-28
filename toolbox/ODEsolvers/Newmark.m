function [up,vp,Jup,Jvp,varargout] = Newmark(M,N,Nu,Nv,F,Fp,T0,T,x0,p,opts,varargin)
% NEWMARK This function returns the final state of IVP: 
%      M\ddot{u}+N(u,\dot{u})=F(t,p) [u0,v0]=x0
% and the sensitivity of final state with respect to x0
% with initial condition x(T0)=x and the sensitivity of final state with
% respect to x0, T0, T and p. The numerical integration used here is
% Newmark method.
%
% [U,V,JU,JV,VARARGOUT] = NEWMARK(M,N,NU,NV,F,FP,T0,T,X0,P,OPTS,VARARGIN)
%
% M:    mass matrix
% N:    function handle for nonlinear internal force N(u,v)
% Nu:   function handle for dNdu, i.e., derivative w.r.t. displacement
% Nv:   function handle for dNdv, i.e., derivative w.r.t. velocity
% F:    function handle for external force F(t,p)
% Fp:   function handle for dFdp
% T0:   initial time
% T:    length of time horizon
% p:    problem parameters
% opts: algorithm parameters for integration
%
% xa and Ja are the final state and its sensitivity respectively. Based on
% the info of varargin, the time history of the system is presented in
% varargout as well.

ItMX   = opts.ItMX;
Nsteps = opts.Nsteps;
RelTol = opts.RelTol;
beta  = 1/4;
gamma = 1/2;
dt   = T/Nsteps;
dim  = numel(x0)/2; % number of dofs
qdim = numel(p);    % number of parameters
up   = x0(1:dim);
vp   = x0(dim+1:2*dim);
ap   = M\(F(T0,p)-N(up,vp));
Jup  = [-vp,eye(dim),zeros(dim),zeros(dim,qdim)];
Jvp  = [-ap,zeros(dim),eye(dim),zeros(dim,qdim)];
f0   = [zeros(dim,1),zeros(dim,2*dim),Fp(T0,p)];
Jap  = M\(f0-Nv(up,vp)*Jvp-Nu(up,vp)*Jup);

if ~isempty(varargin)
    ut = zeros(dim,Nsteps+1);
    ut(:,1) = up;
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
    ik = 1;
    while 1       
        r = M*ac+N(uc,vc)-F(ti,p); % residual
        S = M/(beta*dt^2)+gamma*Nv(uc,vc)/(beta*dt)+Nu(uc,vc);
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
    % update sensitivity matrix
    Justar = Jup+dt*Jvp+(0.5-beta)*dt^2*Jap;
    Jvstar = Jvp+(1-gamma)*dt*Jap;
    S   = M/(beta*dt^2)+gamma*Nv(uc,vc)/(beta*dt)+Nu(uc,vc);
    Fc  = [zeros(dim,1),zeros(dim,2*dim),Fp(ti,p)];
    Juc = S\(Fc+M*Justar/(beta*dt^2)+Nv(uc,vc)*(gamma/(beta*dt)*Justar-Jvstar));
    Jvc = Jvstar+gamma/(beta*dt)*(Juc-Justar);
    Jac = (Juc-Justar)/(beta*dt^2);
    % update results
    up  = uc;
    vp  = vc;
    ap  = ac;
    Jup = Juc;
    Jvp = Jvc;
    Jap = Jac;
    if ~isempty(varargin)
        ut(:,i+1) = up;
    end
end
if ~isempty(varargin)
    varargout{1} = ut;
end

end