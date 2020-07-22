
a = rand(3,4,5);
b = rand(4,6);
c = rand(5,7);


d = einsum(einsum(a,b,2,1),c,2,1);

e = einsum(einsum(a,c,3,1),b,2,1);


Phi_p = rand(3,2);
Phi_T0 = rand(3,1);
aa = einsum(einsum(Fxx, Phi_p, 3, 1), Phi_T0, 2, 1);