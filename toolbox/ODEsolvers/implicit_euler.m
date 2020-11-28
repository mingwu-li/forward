function [xa, Ja] = implicit_euler(fhan,dfdxhan,dfdphan,T0,T,x0,p,isauto,opts)
% IMPLICIT_EULER This function returns the final state of IVP: \dot{x}=f
% with initial condition x(T0)=x and the sensitivity of final state with
% respect to x0, T0, T and p.
%
% [XA, J] = IMPLICIT_EULER(F,FX,FP,T0,T,X0,P,ISAUTO,OPTS)
%
% fhan:    function handle for vector field
% dfdxhan: function handle for dfdx
% dfdphan: function handle for dfdp
% T0:      initial time
% T:       length of time horizon
% p:       problem parameters
% isauto:  flag for vector field is autonomous or not
% opts:    algorithm parameters for integration
%

if isauto
    f  = @(t,x,p) fhan(x,p);
    fx = @(t,x,p) dfdxhan(x,p);
    fp = @(t,x,p) dfdphan(x,p);
else
    f  = fhan;
    fx = dfdxhan;
    fp = dfdphan;
end

ItMX   = opts.ItMX;
Nsteps = opts.Nsteps;
RelTol = opts.RelTol;
dt   = T/Nsteps;
dim  = numel(x0);
qdim = numel(p);
xa   = x0;
Ja   = [-f(T0,x0,p),eye(dim),zeros(dim,qdim)];
for i=1:Nsteps
    ti = T0+i*dt;
    xb = xa; % xa is the solution at previous step
    % xb is the solution at current step (unknown)
    % Newton-Raphson iteration to yield convergent results
    ik = 1;
    while 1
        r = xb-xa-dt*f(ti,xb,p);
        A = eye(dim)-dt*fx(ti,xb,p);
        dx = -A\r;
        xb = xb+dx;
        if norm(dx)<=RelTol*norm(xb)
            % you may further check AbsTol here
            break;            
        end
        assert(ik<ItMX,'Newton-Raphson does not coverge');
        ik = ik+1;
    end
    % update sensitivity matrix
    A  = eye(dim)-dt*fx(ti,xb,p);
    Jb = A\(Ja+[zeros(dim,1),zeros(dim),dt*fp(ti,xb,p)]);
    % update results
    xa = xb;
    Ja = Jb;
end
% Ja(:,1) = Ja(:,1)+f(T0+T,xa,p);

end
            