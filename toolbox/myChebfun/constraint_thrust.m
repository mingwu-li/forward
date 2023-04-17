function [data, y] = constraint_thrust(prob, data, u)

% NOTE: not vectorized

global ncheb

pars = u(1:3*ncheb);

t = u(3*ncheb + 1) + u(3*ncheb + 2);

y = 0;

for i = 1:3
    sum = 0;
    for j = 1:ncheb
       sum = sum + par((i - 1)*ncheb + j).*mychebyshevT(j
        
    end
end








end