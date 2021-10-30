%% 

theta_rem_d = datas(1,:);
theta_rot_d = datas(2,:);
iq_d = datas(3,:);

theta_m = theta_rem_d/(4096);

figure(1)
clf
hold on
plot(time,theta_m);
% plot(iq);
% plot(theta_rem);
% plot(theta_rot);
hold off