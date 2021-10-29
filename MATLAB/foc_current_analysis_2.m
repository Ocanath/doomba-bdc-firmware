%% foc current 2
iq = ctl(1,:);
id = ctl(2,:);
x = ctl(3,:);

figure(1)
clf
hold on
plot(iq);
plot(id);
plot(x);
hold off