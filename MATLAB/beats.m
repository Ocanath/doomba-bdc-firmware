%%

t = 0:1e-7:32e-5;
thresh = .98;

f1 = 32e3 * 2 * pi;
f2 = (1e6/26.5) * 2 * pi;

b1 = sin(f1*t);
b1(b1 > thresh) = 1;
b1(b1 <= thresh) = 0;

b2 = sin(f2*t);
b2(b2 > thresh) = 1;
b2(b2 <= thresh) = 0;

figure(1)
clf
hold on
plot(t, b1)
plot(t, b2)
hold off
%%
t = 0:.001:2;

b1 = sin(t*2*pi*1);
b2 = sin(t*2*pi*4);

b1(b1 > thresh) = 1;
b1(b1 <= thresh) = 0;

b2(b2 > thresh) = 1;
b2(b2 <= thresh) = 0;

figure(1)
clf
hold on
plot(t,b1)
plot(t,b2)
