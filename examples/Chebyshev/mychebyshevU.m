function y=mychebyshevU(j,t)


switch j
    case 0
        y = 1;
    case 1
        y = 2*t;
    case 2
        y = 4*t.^2-1;
    case 3
        y = 8*t.^3-4*t;
    case 4
        y = 16*t.^4-12*t.^2 + 1;
    case 5
        y = 32*t.^5 - 32*t.^3 + 6*t;
    case 6
        y = 64*t.^6 - 80*t.^4 + 24*t.^2 - 1;
    case 7
        y = 128*t.^7 - 192*t.^5 + 80*t.^3 - 8*t;
    case 8
        y = 256*t.^8 - 448*t.^6 + 240*t.^4 - 40*t.^2 + 1;
    case 9
        y = 512*t.^9 - 1024*t.^7 + 672*t.^5 - 160*t.^3 + 10*t;
    otherwise
        disp('please make sure j \in {0,1,2,...,9}');return
end

end