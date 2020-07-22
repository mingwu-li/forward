function y = brat_dx(x,p)

y = [0 1; -p*exp(x(1)) 0];

J = coco_ezDFDX('f(x,p)', @brat, x, p);

end