% a brief demo
% in this version, we use JFNK as nonlinear solver
x0 = zeros(2,1); om0 = 0.1; dom = 0.01; omend = 1.6; R = 0.08;
y = secant_arclength_cont(x0,om0,dom,omend,R,@JFNK);

% calculate amplitude of each periodic orbit
npts = size(y,2);
amp  = zeros(npts,1);
for k = 1:npts
    om = y(end,k);
    T = 2*pi/om;
    odefunc = @(t,x) duffing(t,x,om);
    [~,xt] = ode45(odefunc,[0,T],y(1:end-1,k));
    amp(k) = max(abs(xt(:,1)));
end

figure;
plot(y(end,:),amp,'b-');
xlabel('omega');
ylabel('amp');