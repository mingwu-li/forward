function y = mychebyshevT(j, t)

y = cos(j*acos(t));

if j==0
    y = y/pi^0.5;
else
    y = y/(pi/2)^0.5;
end

end