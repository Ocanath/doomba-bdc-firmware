%% Fixed point controller validity analysis


finger_angle = 45.0;
gbx_ratio = 0.0881744837;
gbx_ratio_fixed_m = int64(gbx_ratio * 2^16);

fang_fixed_rad = int64(floor(finger_angle*(pi/180)*4096 * 20*32.45));

fang_conv = bitshift( (fang_fixed_rad * gbx_ratio_fixed_m), -16);
fang = double(fang_conv)/4096