function y = secant_arclength_cont(x0,om0,dom,omend,R)
% we note that coco use pseudo-arclength method to perform numerical
% continuation. The predictor then asks for tangent direction, which
% involves the jacobi of the zero problem. The computation of this jacobi
% is time-consuming for high-dimensional problems. Here we provide an
% alternative approach which takes secant predictor and arclength as
% corrector


% first point - at om = om0
u0 = [x0;om0];
eq_type1 = @(x) non_eqs(x,[],om0);
u1 = fsolve(eq_type1,u0);

% second point - at om = om0+dom
eq_type1 = @(x) non_eqs(x,[],om0+dom);
u2 = fsolve(eq_type1,u1);

y = [u1 u2];
% continuation until om reaches omend
while 1
    % secant predictor
    u1 = y(:,end-1);
    u2 = y(:,end);
    t  = (u2-u1)/norm(u2-u1);
    data = struct();
    data.ustar = u2;
    data.R = R;
    u0 = u2+t*R;
    eq_type2 = @(x) non_eqs(x,data,[]);
    u3 = fsolve(eq_type2,u0);
    if u3(end)>omend
        break;
    end
    y = [y u3];
end

end


function y = non_eqs(u,data,om0)
x0 = u(1:end-1);
om = u(end);

% forward simulation with one period
T = 2*pi/om;
odefunc = @(t,x) duffing(t,x,om);
[~,xt] = ode45(odefunc,[0,T],x0);
xf = xt(end,:)';
% periodic boundary condition
yp = xf-x0; 
% corrector
if isempty(data)
    y = [yp; om-om0];
else
    ustar = data.ustar;
    yc    = (u-ustar)'*(u-ustar)-data.R^2;
    y     = [yp; yc];
end
end