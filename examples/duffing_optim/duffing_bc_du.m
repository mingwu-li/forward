function [data, J] = duffing_bc_du(prob, data, u) %#ok<INUSD>

J = [
    -1 0 0 1 0 0;
    0 -1 0 0 1 0;
    0 0 -1 0 0 1;
    0 1 0 0 0 0;
    ];

end
