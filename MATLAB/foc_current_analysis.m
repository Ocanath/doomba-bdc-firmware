%% FOC current analysis
load('foc_current_data.mat');
d = data1;

ia = d(1,:);
ib = d(2,:);
ic = d(3,:);

figure(1)
clf
hold on
plot(ia);
plot(ib);
plot(ic);
hold off