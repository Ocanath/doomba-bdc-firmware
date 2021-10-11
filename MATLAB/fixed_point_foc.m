%% Fixed point strategy:

% core data in is 12bit adc. We run on a 32 bit processor, so we will use 
% 32 bits only operations to avoid implicit casting.
% 

%%

theta = (-pi-1):.001:(pi+1);
adc_sin = int32(sin(theta)*2018+1975);
adc_cos = int32(cos(theta)*2018+1975);

sin_in = adc_sin - int32(1975);
cos_in = adc_cos - int32(1975);


ref = atan2(sin(theta),cos(theta));
float_ref = atan2(single(sin_in),single(cos_in));
fixed_ref = int32(zeros(1,length(theta)));
for i = 1:length(theta)
    fixed_ref(i) = atan2_fixed(int32(sin_in(i)),int32(cos_in(i)));
end
% fixed_ref = atan2_fixed(sin_in,cos_in);



% disp(max(fixed_ref));

figure(1)
clf
hold on
% plot(int32(sin_in));
% plot(int32(cos_in));
plot(double(fixed_ref)/4096);
% plot(ref);
plot(float_ref);
hold off

figure(2)
clf
err = float_ref-double(fixed_ref)/4096;
plot(err);
fprintf("max err in degrees = %f\n",max(abs(err))*(360/pi));

%% Basis for computation of fast fixed point '12 bit' atan2


in = 0:1/4096:1;

ang_out = atan(in);
c = polyfit(in,ang_out,4)*4096;  %get 4th order poly approximation
atanfit = c(1)*in.^4 + c(2)*in.^3 + c(3)*in.^2 + c(4)*in + c(5);

% Compute the second, third, and fourth exponents of the input
a = int32(floor(in*4096));  
a2 = bitshift(a.*a,-12);
a3 = bitshift(a2.*a,-12);
a4 = bitshift(a3.*a,-12);   %a3 *a instead of a2*a2 bc of the bitshifting

% fp_coef = int32(round(c));
fp_coef = int32([580, -1406, -66, 4112, -1]);

%ok so that's three multiplies and 3 bitshifts so far

fpc = int32(a4*fp_coef(1) + a3*fp_coef(2) + a2*fp_coef(3) + a*fp_coef(4) + fp_coef(5));
fpc = bitshift(fpc, -12);   %extra factor of 2^12 in coef, so shift it back
fprintf("fpcmax = %f\n",max(fpc));
err = max(abs(double(fpc)/4096 - ang_out))*(360/pi);
fprintf("max direct = %f degrees\n", err);

csecond = int32(round(polyfit(in,ang_out,2)*4096)); %this yields the nth order polynomial coefficients of the poly that approximates the input curve, scaled for 12 bit fixed point
fpc2 = a2*csecond(1) + a*csecond(2) + csecond(3);
fpc2 = bitshift(fpc2,-12);
err = max(abs(double(fpc2)/4096 - ang_out))*(360/pi);
fprintf("second = %f degrees\n", err);

cthird = int32(round(polyfit(in,ang_out,3)*2^12));
fpc3 = a3*cthird(1) + a2*cthird(2) + a*cthird(3) + cthird(4);
fpc3 = bitshift(fpc3,-12);
err = max(abs(double(fpc3)/4096 - ang_out))*(360/pi);
fprintf("third = %f degrees\n", err);

n = 4;
nth_fixed_approx = polyval(  floor(polyfit(in,ang_out, n)*4096)    ,in)/4096;
fprintf("worst case error order %d = %f\n", n, (max(abs(ang_out-nth_fixed_approx))));


baseline_poly = -0.0464964749*in.^7 + 0.15931422*in.^5 - 0.327622764*in.^3 + in;

figure(1)
clf
hold on
plot(in,ang_out);
% plot(in,nth_fixed_approx);
plot(in,single(fpc)/4096);
plot(in,baseline_poly);
hold off

