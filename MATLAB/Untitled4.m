%%
current = 1.2345;   %real life current!
current_digitized = int32(floor(current*525.9366)); %convert the current into adc digits (offsetting is ignored here)

current_digitized = bitshift(current_digitized,1)+1;    %do half lsb centering to reduce error by a teensy amount
cd_norm = 31900*current_digitized;  % apply scaling factor. Brings qty to units*2^25
cd_norm_12b = bitshift(cd_norm, -13);    %reduce to units of current*2^12

cd_measured = double(cd_norm_12b)/2^12

%%

scf = [1/525.9366, 1/340.4033, 1/185.8066];
scf_norm = scf/(sqrt(sum(scf.^2)));
scf_d = int32(scf*2^24);

ia = sin(0:.001:3)*2048;

approx_12b = double (bitshift(int32(ia)*scf_d(3),-12))/4096.0;
approx_24b = double(int32(ia)*scf_d(3))/2^24;
true_val = scf(3)*ia;


err12 = abs(true_val-approx_12b);
err24 = abs(true_val-approx_24b);

figure(2)
clf
hold on
plot(err12);
plot(err24);
hold off

%%
id = iq_1000l(2,:);
iq = iq_1000l(1,:);
theta = iq_1000l(3,:);

figure(1)
clf;
hold on
plot(theta)
plot(iq)
plot(id)

% plot(iq+id)

% plot(.1e4*sin(double(theta*2)/4096 - 0.65) - .4e4);
% plot(.1e4*sin(double(theta*2)/4096 - 0.65 - pi/2) );
hold off

%%
plot(iq_f)