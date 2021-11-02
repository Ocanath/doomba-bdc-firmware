t = 0:.1:1000000;
theta = -5*t+pi;
theta(round(end/3):end) = -theta(round(end/3):end);
theta = mod(theta+pi,2*pi)-pi;
theta = int32(theta*4096);

rem = int32( zeros(1,length(t)) );
rot_log = rem;
rotations = int32(0);
prev = int32(0);
half_ovfl_val = 41721*int32(pi*4096);
for i=1:length(t)
    [rem(i),prev, rotations] = unwrap32_fixed(theta(i),prev, rotations, half_ovfl_val);
    rot_log(i) = rotations;
end
figure(2)
clf
hold on
plot(t,double(theta)/(4096*pi));
plot(t, (double(rem) + double(rot_log)*2*double(half_ovfl_val))/(4096*pi) );
plot(t,double(rem)/(4096*pi));
plot(t,rot_log*40000);
hold off
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