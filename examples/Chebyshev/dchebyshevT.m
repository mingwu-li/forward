function y = dchebyshevT(j,t)

% y = (j*sin(j*acos(t)))./(1 - t.^2).^(1/2); % 0/0 at t=-1 and t=1
y = j*mychebyshevU(j-1,t);
y = y/(pi/2)^0.5;

end