function [data, y] = amplitude(~, data, u)

N  = data.dim;
x0 = u(1:N);
T0 = u(N+1);
T  = u(N+2);
p  = u(N+3:end);

% evaluate amplitudes at outdofs AND stability of periodic orbit
[~,~,Jup,Jvp,zt] = Newmark(data.M, data.Nhan, data.dNdu, data.dNdv, ...
    data.dNdp, data.Fext, data.dFextdp, T0, T, x0, p, data.opts, 'var', data.dof);
% rewrite results in first-order form
y = max(abs(zt),[],2);
Jx0     = [Jup(:,2:data.dim+1); Jvp(:,2:data.dim+1)];
leadEig = eigs(Jx0,1);
stab    = double(abs(leadEig)<1);

y = [y; stab];
end