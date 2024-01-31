%% Fixed point strategy:

% core data in is 12bit adc. We run on a 32 bit processor, so we will use 
% 32 bits only operations to avoid implicit casting.
% 
t = 0:1e-6:.03;
f = 90*2*pi;
theta = t*f;
mag = 300;
[a,b] = inverse_park(mag,0,sin(theta),cos(theta));
[ia,ib,ic] = inverse_clarke(a,b);
ia = int32(ia);
ib = int32(ib);
ic = int32(ic);

[ialpha,ibeta] = clarke_fixed(ia,ib,ic);
sin_theta = int32(sin(theta+offset)*4096);
cos_theta = int32(sin(theta+offset)*4096);
iq = zeros(1,length(theta));
id = zeros(1,length(theta));
for i = 1:length(theta)
    [iq(i),id(i)] = park_fixed(ialpha(i),ibeta(i),sin_theta(i),cos_theta(i));    
end


figure(1)
clf;
hold on
% plot(ia);
% plot(ib);
% plot(ic);
% plot(ialpha);
% plot(ibeta);
plot(iq);
plot(id);
hold off


%% inverse park + SVM check
% theta = (-pi-1):.001:(pi+1);
% theta = T.theta_gen;
t = 0:.001:10;
theta = t.^3/4-5*pi;

PI                      =   int32(12868);    %int32(3.14159265359*4096);
TWO_PI = 2*PI;

align_offset = 0.2;
align_offset_fixed = int32(align_offset *4096); %compute fixed align offset from floating point one
elec_conv_ratio = int32(7);
iq = int32(sqrt(3)/2*2^12);
id = int32(0);

%generate fake encoder signal
adc_sin = int32(sin(theta)*2018+1975);
adc_cos = int32(cos(theta)*2018+1975);
sin_in = adc_sin - int32(1975);
cos_in = adc_cos - int32(1975);

%allocate logger memory
sin_f = int32(zeros(1,length(theta)));
cos_f = int32(zeros(1,length(theta)));
thm = int32(zeros(1,length(theta)));
tA = int32(zeros(1,length(theta)));
tB = int32(zeros(1,length(theta)));
tC = int32(zeros(1,length(theta)));

prev_theta_m = int32(0);
phase_adv = int32(0);
get_v_ts = uint32(0);
v_m = zeros(1,length(theta));
%foc loop
for i = 1:length(theta)
    tick = uint32(t(i)*1000);
    
    theta_m = atan2_12b(sin_in(i),cos_in(i)) - align_offset_fixed;
    theta_elec = mod( (theta_m*elec_conv_ratio + phase_adv) +PI, TWO_PI)-PI;
    sin_f(i) = sin_fixed(theta_elec);
    cos_f(i) = cos_fixed(theta_elec);   
    
    [alpha,beta] = inverse_park_fixed(iq,id,sin_f(i),cos_f(i));
    [tA(i),tB(i),tC(i)] = svm_fixed(alpha,beta,int32(1000));
    
    if(tick >= get_v_ts)
        v_m(i) = mod( (theta_m - prev_theta_m) + PI,TWO_PI)-PI;
%         v_m(i) = theta_m - prev_theta_m;
        phase_adv = v_m(i)/int32(28);   %implicitly multiply this by 25
        prev_theta_m = theta_m;
        get_v_ts = tick() + 40;
    elseif(i > 2)   %do this so we can check vm calculation
        v_m(i) = v_m(i-1);
    end
    thm(i) = theta_m;
end

figure(1)
clf
hold on
plot(sin_in*2);
plot(cos_in*2);
plot(sin_f);
plot(cos_f);
plot(tA);
plot(tB);
plot(tC);
xlim([1,3000])
hold off

figure(2)
clf
hold on
plot(v_m);
plot(thm);
plot(v_m/28);
hold off
% [ialpha,ibeta] = inverse_park_fixed(iq,id,

%% Atan2+Sin+Cos with encoder quantization check
PI                      =   int32(12868);    %int32(3.14159265359*4096);
TWO_PI = 2*PI;

theta = (-pi-1):.001:(pi+1);


adc_sin = int32(sin(theta)*2018+1975);
adc_cos = int32(cos(theta)*2018+1975);
sin_in = adc_sin - int32(1975);
cos_in = adc_cos - int32(1975);

align_offset = 0;   %0 for direct theta map

fixed_theta_shifted = int32(zeros(1,length(theta)));
sin_fixed_shifted = fixed_theta_shifted;
cos_fixed_shifted = sin_fixed_shifted;

for i = 1:length(theta)
    th = atan2_12b(sin_in(i),cos_in(i));
    th = mod((th+align_offset+PI),TWO_PI)-PI;
    sth = sin_fixed(th);
    cth = cos_fixed(th);
    sin_fixed_shifted(i) = sth;
    cos_fixed_shifted(i) = cth;
    fixed_theta_shifted(i) = atan2_12b(sth/2,cth/2);
end

figure(6)
clf
err = (mod(theta+pi,2*pi)-pi)-double(fixed_theta_shifted)/4096;
err(abs(err)>6) = [];
plot(err);
fprintf('max overall = %f\n',max(abs(err))*360/pi);

figure(5)
clf
hold on
plot((mod(theta+pi,2*pi)-pi)*4096);
plot(sin_in);
plot(cos_in);
plot(sin_fixed_shifted/2);
plot(cos_fixed_shifted/2);
plot(fixed_theta_shifted);
plot(err*4096*pi/max(err));
hold off


figure(7)
clf
hold on
plot(sin_in);
plot(cos_in);
plot(sin_fixed_shifted/2);
plot(cos_fixed_shifted/2);
hold off

%% Atan2 performance / accuracy check
theta = (-pi-1):.001:(pi+1);


adc_sin = int32(sin(theta)*2018+1975);
adc_cos = int32(cos(theta)*2018+1975);

sin_in = adc_sin - int32(1975);
cos_in = adc_cos - int32(1975);


ref = atan2(sin(theta),cos(theta));
float_ref = atan2(single(sin_in),single(cos_in));
fixed_ref = int32(zeros(1,length(theta)));
for i = 1:length(theta)
    fixed_ref(i) = atan2_12b(int32(sin_in(i)),int32(cos_in(i)));
end
% fixed_ref = atan2_12b(sin_in,cos_in);



% disp(max(fixed_ref));

figure(1)
clf
hold on
plot(int32(sin_in));
plot(int32(cos_in));
plot(double(fixed_ref));
% plot(ref);
plot(float_ref*4096);
hold off

figure(2)
clf
err = float_ref-double(fixed_ref)/4096;
plot(err);
fprintf('max err in degrees = %f\n',max(abs(err))*(360/pi));

%% sin and cos check
theta = (-pi-1):.001:(pi+1);

theta_fixed = int32(floor(theta*4096));

sth = sin(theta);
sin_fixed_theta = int32(zeros(1,length(theta)));
cos_fixed_theta = sin_fixed_theta;
for i = 1:length(theta)
    thf = mod(theta_fixed(i)+int32(4096*pi),int32(2*pi*4096))-int32(4096*pi);
    sin_fixed_theta(i) = sin_fixed(thf);
    cos_fixed_theta(i) = cos_fixed(thf);
end
figure(3)
clf
hold on
plot(theta/pi,sth);
plot(theta/pi,double(cos_fixed_theta)/4096);
plot(theta/pi,double(sin_fixed_theta)/4096);
hold off

err = double(sin_fixed_theta)/4096 - sth;
fprintf('max err %f\n',max(abs(err)) );
figure(4)
clf
hold on
plot(err);
hold off
%% sanity check, delete
theta = -pi/2:.001:0;
figure(1)
clf
plot(theta/pi,(theta+pi/2)/pi);

%% Sin and cos alt basis
theta = 0:.001:pi/2;

c = polyfit(theta,sin(theta),4);
cf = int32(round(c*2^15))
pv_fit = polyval(c,theta);

taylor = theta-theta.^3*(1/factorial(3)) + theta.^5*(1/factorial(5));

figure(1)
clf
hold on
plot(sin(theta));
plot(taylor);
plot(pv_fit);
hold off

figure(2)
clf
hold on
plot(sin(theta)-taylor);
plot(sin(theta)-pv_fit);
hold off


%% Basis for computation of fast fixed point '12 bit' atan2


in = 0:1/4096:1;

ang_out = atan(in);
c = polyfit(in,ang_out,4)*2^15;  %get 4th order poly approximation
atanfit = c(1)*in.^4 + c(2)*in.^3 + c(3)*in.^2 + c(4)*in + c(5);

% Compute the second, third, and fourth exponents of the input
a = int32(floor(in*4096));  
a2 = bitshift(a.*a,-12);
a3 = bitshift(a2.*a,-12);
a4 = bitshift(a3.*a,-12);   %a3 *a instead of a2*a2 bc of the bitshifting

fp_coef = int32(round(c));
% fp_coef = int32([580, -1406, -66, 4112, -1]);

%ok so that's three multiplies and 3 bitshifts so far

fpc = int32(a4*fp_coef(1) + a3*fp_coef(2) + a2*fp_coef(3) + a*fp_coef(4) + fp_coef(5));
fpc = bitshift(fpc, -12);   %extra factor of 2^12 in coef, so shift it back
fprintf('fpcmax = %f\n',max(fpc));
err = max(abs(double(fpc)/4096 - ang_out))*(360/pi);
fprintf('max direct = %f degrees\n', err);

csecond = int32(round(polyfit(in,ang_out,2)*4096)); %this yields the nth order polynomial coefficients of the poly that approximates the input curve, scaled for 12 bit fixed point
fpc2 = a2*csecond(1) + a*csecond(2) + csecond(3);
fpc2 = bitshift(fpc2,-12);
err = max(abs(double(fpc2)/4096 - ang_out))*(360/pi);
fprintf('second = %f degrees\n', err);

cthird = int32(round(polyfit(in,ang_out,3)*2^12));
fpc3 = a3*cthird(1) + a2*cthird(2) + a*cthird(3) + cthird(4);
fpc3 = bitshift(fpc3,-12);
err = max(abs(double(fpc3)/4096 - ang_out))*(360/pi);
fprintf('third = %f degrees\n', err);

% cfifth = int32(round(polyfit(in,ang_out,5)*2^12));
cfifth_factor15 = int32(round(polyfit(in,ang_out,5)*2^15));


n = 4;
nth_fixed_approx = polyval(  floor(polyfit(in,ang_out, n)*4096)    ,in)/4096;
fprintf('worst case error order %d = %f\n', n, (max(abs(ang_out-nth_fixed_approx))));


baseline_poly = -0.0464964749*in.^7 + 0.15931422*in.^5 - 0.327622764*in.^3 + in;

figure(1)
clf
hold on
plot(in,ang_out);
% plot(in,nth_fixed_approx);
plot(in,single(fpc)/4096);
plot(in,baseline_poly);
hold off


%%


