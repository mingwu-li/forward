clc
clear

ntems = 2:9;
nruns = numel(ntems);
y = zeros(2,nruns);
for k=1:nruns
    y(:,k) = opt_control_ivp(ntems(k));
end

figure(2)
semilogy(ntems,y(1,:),'ro-');
set(gca,'LineWidth',1.5);
set(gca,'FontSize', 14);
xlabel('Expansion terms $q$','interpreter','laTex')
ylabel('$|| u(t)-u_{\mathrm{exact}}(t) ||_{[0,2],2}$','interpreter','laTex')

figure(3)
semilogy(ntems,y(2,:),'bv-');
set(gca,'LineWidth',1.5);
set(gca,'FontSize', 14);
xlabel('Expansion terms $q$','interpreter','laTex')
ylabel('$|J-J_{\mathrm{exact}}|/|J_{\mathrm{exact}}|$','interpreter','laTex')


