%%
deg = 60*3;
theta = int32(deg * pi * 4096 / 180);
cth = cos_fixed(theta);
sth = sin_fixed(theta);

[alpha,beta]=inverse_park_fixed(int32(200),int32(0),sth,cth);
[ta,tb,tc] = svm_fixed(alpha,beta,int32(1000));