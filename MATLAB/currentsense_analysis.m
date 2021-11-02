t = 0:0.001:3;
ft = 5*t;
ia = int32(4096*(.5*sin(ft+0*2*pi/3)+.5));
ib = int32(4096*(.5*sin(ft+1*2*pi/3)+.5));
ic = int32(4096*(.5*sin(ft+2*2*pi/3)+.5));

ia = bitshift(ia-2048,1)+1;
ib = bitshift(ib-2048,1)+1;
ic = bitshift(ic-2048,1)+1;

ia = bitshift(31900*ia,-13);
ib = bitshift(31900*ib,-13);
ic = bitshift(31900*ic,-13);

[alpha,beta] = clarke_fixed(ia,ib,ic);

sth = int32(4096*sin(theta));
cth = int32(4096*cos(theta));
[iq,id] = park_fixed(alpha,beta,sth,cth);
figure(2)
clf
hold on
plot(ia);
plot(ib);
plot(ic);
plot(alpha);
plot(beta);
plot(iq);
plot(id);
hold off

maxes = [max(abs(iq)) max(abs(id))];
num_bits(max(maxes))

%%
figure(1)
clf
hold on
% plot(datas_iq_0p5);
% plot(datas_iq_0p859);
% plot(datas_iq_0p5_loaded);
% plot(datas_iq_0p5_loaded2);
plot(datas_iq_0p1_loaded);
plot(datas_iq_0p1_loaded2);
plot(datas_iq_0p1_loaded3);
hold off