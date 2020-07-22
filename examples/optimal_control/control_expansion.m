function u = control_expansion(t,p)

global ncheb
u = 0;
for j=1:ncheb
    u = u + p(j)*mychebyshevT(j-1,t);
end

end
