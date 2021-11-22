%%

dq_output = datas(1,:);
dq_rotor = datas(2,:);
q_output = datas(3,:);
u = datas(4,:);

points = 20;
B = (1/points)*ones(points,1);
dqout_filt = filter(B,1,dq_output);
dqrotor_filt = filter(B,1,dq_rotor);

dq = wrap(q_output(2:end)-q_output(1:end-1),2*pi);
% dt = (time(2:end) - time(1:end-1));
dt = 5.3128/1000*ones(1,length(dq));
dq_output_calc = dq./dt

figure(5)
clf
hold on
plot(wrap(q_output,2*pi));
plot(q_output);
plot(dq_output_calc)
plot(dqout_filt);
hold off

figure(3)
clf
hold on
plot(dq_output);
plot(dqout_filt);
plot(dq_rotor);
plot(dqrotor_filt);
hold off

figure(2)
clf
hold on
plot(dqout_filt);
plot(-dqrotor_filt/16);
hold off


figure(4)
clf
hold on
plot(dqout_filt);
plot(dq_output_calc);
plot(-dq_rotor/16);
hold off

figure(6)
plot(dt)


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