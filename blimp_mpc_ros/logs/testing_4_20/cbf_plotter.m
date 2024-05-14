clear
close all
clc

data = readmatrix('linecbf2.csv');

tt = data(:, 1);
v_x__b = data(:, 2);
v_y__b = data(:, 3);
v_z__b = data(:, 4);
w_x__b = data(:, 5);
w_y__b = data(:, 6);
w_z__b = data(:, 7);
x = data(:, 8);
y = data(:, 9);
z = data(:, 10);
phi = data(:, 11);
theta = data(:, 12);
psi = data(:, 13);
fx = data(:, 26);
fy = data(:, 27);
fz = data(:, 28);
tau_z = data(:, 29);

theta_limit = 10 * pi./180;
gamma_th = 1;

m_x = 0.1715;
I_x = 0.00613325;
m_RB = 0.1249;
r_z_gb__b = 0.05;
I_y = 0.00613325;
m_y = 0.1715;
D_vxy__CB = 0.0125;
m_z = 0.1794;
D_omega_xy__CB = 0.000862;
I_z = 0.005821;
f_g = 1.22402;
D_omega_z__CB = 0.000862;
r_z_tg__b = 0.245;

h_th = 1./2 .* (-theta.^2 + theta_limit.^2);
psi1_th = - theta.*(cos(phi).*w_y__b - sin(phi).*w_z__b) + gamma_th.*h_th;

lfpsi1_th = theta.*(cos(phi).*w_z__b + sin(phi).*w_y__b).*(w_x__b + cos(phi).*tan(theta).*w_z__b + sin(phi).*tan(theta).*w_y__b) - (cos(phi).*w_y__b - sin(phi).*w_z__b).*(cos(phi).*w_y__b - sin(phi).*w_z__b + gamma_th.*theta) + cos(phi).*theta.*(w_z__b.*((m_x.*(I_x.*w_x__b - m_RB.*r_z_gb__b.*v_y__b))./(I_y.*m_x - m_RB.^2.*r_z_gb__b.^2) + (m_RB.*r_z_gb__b.*(m_y.*v_y__b - m_RB.*r_z_gb__b.*w_x__b))./(I_y.*m_x - m_RB.^2.*r_z_gb__b.^2)) - v_x__b.*((D_vxy__CB.*m_RB.*r_z_gb__b)./(I_y.*m_x - m_RB.^2.*r_z_gb__b.^2) + (m_x.*m_z.*v_z__b)./(I_y.*m_x - m_RB.^2.*r_z_gb__b.^2)) + w_y__b.*((D_omega_xy__CB.*m_x)./(I_y.*m_x - m_RB.^2.*r_z_gb__b.^2) - (m_RB.*m_z.*r_z_gb__b.*v_z__b)./(I_y.*m_x - m_RB.^2.*r_z_gb__b.^2)) + (m_x.*v_z__b.*(m_x.*v_x__b + m_RB.*r_z_gb__b.*w_y__b))./(I_y.*m_x - m_RB.^2.*r_z_gb__b.^2) - (I_z.*m_x.*w_x__b.*w_z__b)./(I_y.*m_x - m_RB.^2.*r_z_gb__b.^2) + (f_g.*m_x.*r_z_gb__b.*sin(theta))./((I_y.*m_x - m_RB.^2.*r_z_gb__b.^2).*(cos(theta).^2 + sin(theta).^2))) - sin(phi).*theta.*((w_x__b.*(I_y.*w_y__b + m_RB.*r_z_gb__b.*v_x__b))./I_z - (w_y__b.*(I_x.*w_x__b - m_RB.*r_z_gb__b.*v_y__b))./I_z - (v_y__b.*(m_x.*v_x__b + m_RB.*r_z_gb__b.*w_y__b))./I_z + (v_x__b.*(m_y.*v_y__b - m_RB.*r_z_gb__b.*w_x__b))./I_z + (D_omega_z__CB.*w_z__b)./I_z);
lgpsi1_th = ...
    [cos(phi).*theta.*((m_RB.*r_z_gb__b)./(I_y.*m_x - m_RB.^2.*r_z_gb__b.^2) - (m_x.*r_z_tg__b)./(I_y.*m_x - m_RB.^2.*r_z_gb__b.^2)) zeros(length(tt), 1) zeros(length(tt), 1) (sin(phi).*theta)./I_z];

mu = [fx fy fz tau_z].';

psi_dot = NaN(1, length(tt));

psi_dot_empirical = [0 diff(psi1_th).'/0.05];

for t = 1:length(tt)
    psi_dot(t) = lfpsi1_th(t) + lgpsi1_th(t, :) * mu(:, t);
end

yyaxis left
plot(tt, psi_dot)
hold on
yline(0)
ylabel('psi dot from experiment')
title('psi dot')
ylim([-5 5])
yyaxis right
plot(tt, psi_dot_empirical);
ylabel('psi dot from matlab')
ylim([-5 5])
xlabel('time')