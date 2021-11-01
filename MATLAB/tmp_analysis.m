%%

[datas,time] = plot_lines(3,'int32',921600,3000,-10000,10000);
 %%
 
iq = datas(1,:);
id = datas(2,:);
theta = datas(3,:);

figure(2)
clf
hold on
plot(iq);
plot(id);
hold off

figure(3)
clf
plot(time,theta/(4096*2*pi));


v_rot_per_sec = ((theta(end)-theta(1))/(time(end)-time(1)))/(4096*2*pi);
%%

figure(2)
clf
hold on
plot(iq_s);
plot(iq_foc);
hold off
legend('i_q sinusoidal','i_q foc');
title('I_q ripple comparison');

figure(3)
clf
hold on
plot(id_s);
plot(id_foc);
hold off
legend('i_d sinusoidal','i_d foc');
title('I_d ripple comparison ');