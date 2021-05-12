function J = duffing_seq_dp(t, x, p)

x1 = x(1,:);
amp   = p(1,:);
omega = p(2,:);

J = zeros(4,2,numel(x1));
J(2,1,:) = cos(omega.*t);
J(2,2,:) = -amp.*t.*sin(omega.*t);
J(4,1,:) = cos(omega.*t);
J(4,2,:) = -amp.*t.*sin(omega.*t);

end
