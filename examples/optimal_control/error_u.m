function y = error_u(t,p)

global ncheb
u1 = 0;
for j=1:ncheb
    u1 = u1 + p(j)*mychebyshevT(j-1,t-1);
end
u2 = 2./(1+3*exp(5*t/2));
y = (u1-u2).^2;

end