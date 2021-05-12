function J = doedel_seq_DFDP(x, p) %#ok<INUSD>
%DOEDEL_DFDP   'coll'-compatible encoding of Jacobian with respect to parameters

x1 = x(1,:);
x2 = x(2,:);
x3 = x(3,:);
x4 = x(4,:);

J = zeros(4,2,numel(x1));
J(2,1,:) = x1;
J(2,2,:) = x2;
J(4,1,:) = x3;
J(4,2,:) = x4;
end