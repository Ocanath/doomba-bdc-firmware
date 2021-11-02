%%
% [datas,time] = plot_lines(2,'int32',921600,5000,-4096,4096);

iq = datas(1,:);
id = datas(2,:);

figure(1)
clf
hold on
plot(iq);
plot(id);
hold off

aiq_foc = abs(iq(1:3000));
aid_foc = abs(id(1:3000));

aiq_s = abs(iq(3001:end));
aid_s = abs(id(3001:end));

figure(2)
clf
hold on
plot(aiq_s);
plot(aid_s);
plot(aiq_foc);
plot(aid_foc);
hold off

iq_redux = mean(aiq_s)/mean(aiq_foc)
id_redux = mean(aid_s)/mean(aid_foc)

total_redux = (mean(aiq_s) + mean(aid_s))/(mean(aid_foc)+mean(aid_foc))

%% FOC current analysis
va=IV(4,:);
vb=IV(5,:);
vc=IV(6,:);

windowSize = 50; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;

va_filt = filter(b,a,va);
vb_filt = filter(b,a,vb);
vc_filt = filter(b,a,vc);

ia = IV(1,:);
ib = IV(2,:);
ic = IV(3,:);

iafilt = filter(b,a,ia);
ibfilt = filter(b,a,ib);
icfilt = filter(b,a,ic);

vscf = 10;
figure(1);
clf;
hold on
% plot(IV(1,:));
% plot(IV(2,:));
% plot(IV(3,:));
plot(iafilt);
plot(ibfilt);
plot(icfilt);
% plot(va_filt*vscf);
% plot(vb_filt*vscf);
% plot(vc_filt*vscf);
plot(iafilt+ibfilt+icfilt);
hold off

figure(2)
clf
hold on
% plot(IV(4,:));
% plot(IV(5,:));
% plot(IV(6,:));

plot(va);
plot(vb);
plot(vc);



hold off



%% ANALYSIS ON DATASET WITH POSITIVE AND NEGATIVE STEADY STATE CURRENTS
%in each test, the voltage was set to 'active = (+/-200)+562', and
%'non-active = 0+562'. The raw ADC was left shifted + 1, and then div by
%4096. To obtain the raw ADC value, multiply by 4096, subtract 1, divide by
%2.


% To get the weights:
%     1. send a test voltage va=x,vb=0,vc=0
%     2. obtain the average current over some period of time, once the current is
%         reached steady state
%     3. V1 = [i_avg_a, i_avg_b, i_avg_c]
%     4. Repeat with va = 0, vb = x, vc = 0 to obtain V2
%     5. [weighta, weightb, weightc] = K*cross(V1,V2). K is the constant that 
%         makes these weights give a true current measurement
%     6. repeat these tests for multiple variations of x (sign and value).
%         You should see consistency across tests. 6 tests was enough
%         on my single channel FOC v4-r5 boards
%     7. Take the average weights across all tests

load('foc_steadystate_current_pn_data.mat');

dataset = {data1, data2, data3, data4, data5, data6};
means = {zeros(1,3),zeros(1,3),zeros(1,3),zeros(1,3),zeros(1,3),zeros(1,3)};
weight_a = 1;
weight_b = 1;
weight_c = 1;
% weight_a = 1/normal(1);
% weight_b = 1/normal(2);
% weight_c = 1/normal(3);
for i = 1:size(dataset,2)
    
    d = dataset{i};
    
    ia = d(1,:)*weight_a;
    ib = d(2,:)*weight_b;
    ic = d(3,:)*weight_c;
    
    means{i} = [mean(ia),mean(ib),mean(ic)];
    
    figure(i)
    clf
    hold on
    plot(ia);
    plot(ib);
    plot(ic);
    plot(ones(1,length(ia))*means{i}(1));
    plot(ones(1,length(ia))*means{i}(2));
    plot(ones(1,length(ia))*means{i}(3));
    hold off
    legend('ia','ib','ic','meanA','meanB','meanC');
end

mean_set1= [means{1}; means{2}; means{3}];
mean_set2 = [means{4}; means{5}; means{6}];
x = -1:.001:1;
y=x;
z=y;

figure(10)
clf
hold on

v1 = [mean_set1(1, 1), mean_set1(1, 2), mean_set1(1,3)];
v2 = [mean_set1(2, 1), mean_set1(2, 2), mean_set1(2,3)];
normal = cross(v2, v1);
normal1 = normal;
[xp,yp] = meshgrid(-1:0.1:1);
zp = -(1/normal(3)) * (normal(1)*xp + normal(2)*yp );
surf(xp,yp,zp);

v1 = [mean_set1(2, 1), mean_set1(2, 2), mean_set1(2,3)];
v2 = [mean_set1(3, 1), mean_set1(3, 2), mean_set1(3,3)];
normal = cross(v2, v1);
normal2 = normal;
[xp,yp] = meshgrid(-1:0.1:1);
zp = -(1/normal(3)) * (normal(1)*xp + normal(2)*yp );
surf(xp,yp,zp);

v1 = [mean_set1(1, 1), mean_set1(1, 2), mean_set1(1,3)];
v2 = [mean_set1(3, 1), mean_set1(3, 2), mean_set1(3,3)];
normal = cross(v2, v1);
normal3 = normal;
[xp,yp] = meshgrid(-1:0.1:1);
zp = -(1/normal(3)) * (normal(1)*xp + normal(2)*yp );
surf(xp,yp,zp);

v1 = [mean_set2(1, 1), mean_set2(1, 2), mean_set2(1,3)];
v2 = [mean_set2(2, 1), mean_set2(2, 2), mean_set2(2,3)];
normal = cross(v2, v1);
normal4 = normal;
[xp,yp] = meshgrid(-1:0.1:1);
zp = -(1/normal(3)) * (normal(1)*xp + normal(2)*yp );
surf(xp,yp,zp);

v1 = [mean_set2(2, 1), mean_set2(2, 2), mean_set2(2,3)];
v2 = [mean_set2(3, 1), mean_set2(3, 2), mean_set2(3,3)];
normal = cross(v2, v1);
normal5 = normal;
[xp,yp] = meshgrid(-1:0.1:1);
zp = -(1/normal(3)) * (normal(1)*xp + normal(2)*yp );
surf(xp,yp,zp);

v1 = [mean_set2(1, 1), mean_set2(1, 2), mean_set2(1,3)];
v2 = [mean_set2(3, 1), mean_set2(3, 2), mean_set2(3,3)];
normal = cross(v2, v1);
normal6 = normal;
[xp,yp] = meshgrid(-1:0.1:1);
zp = -(1/normal(3)) * (normal(1)*xp + normal(2)*yp );
surf(xp,yp,zp);

% for i = 1:3
%     plot3(mean_set1(i,1)*x,mean_set1(i,2)*y,mean_set1(i,3)*z);
% end
% for i = 1:3
%     plot3(mean_set2(i,1)*x,mean_set2(i,2)*y,mean_set2(i,3)*z);
% end
% for i = 1:3
%     plot3(mean_set3(i,1)*x,mean_set3(i,2)*y,mean_set3(i,3)*z);
% end
hold off
% for i = 1:length(lines)
%     v = [x(i), y(i), z(i)]';
%     lines(:,i) = mean_set1*v;
% end
w1 = (1./normal1)*525.9366/(1./normal1(1));
w2 = (1./normal2)*525.9366/(1./normal2(1));
w3 = (1./normal3)*525.9366/(1./normal3(1));
w4 = (1./normal4)*525.9366/(1./normal4(1));
w5 = (1./normal5)*525.9366/(1./normal5(1));
w6 = (1./normal6)*525.9366/(1./normal6(1));

M = [w1; w2; w3; w4; w5; w6;];
mean(M)

%% ANALYSIS ON DATASET WITH SINUSOIDAL 3 PHASE TRACKING AND POSITIVE STEADY STATE CURRENT
load('foc_current_data.mat');
dataset = {data1, data2, data3, data4, data5, data6};
%
mean_cur = zeros(3,3);
conv_a = 525.9366;
conv_b = 340.4033;
conv_c = 185.8066;
for i = 4:6
    d = dataset{i};
    ia = d(1,:)*conv_a;
    ib = d(2,:)*conv_b;
    ic = d(3,:)*conv_c;
    mean_cur(i-3,:) = abs([mean(ia),mean(ib),mean(ic)]);
end
B=[mean(1,1)*2, mean(1,2)*2 mean(1,3)*2]';
sols = linsolve(mean_cur,B)
% sols = null(mean_cur)


x = -1:.001:1;
y = -1:.001:1;
z = -1:.001:1;
figure(99)
clf
hold on
plot3(mean_cur(1,1)*x,mean_cur(1,2)*y,mean_cur(1,3)*z);
plot3(mean_cur(2,1)*x,mean_cur(2,2)*y,mean_cur(2,3)*z);
plot3(mean_cur(3,1)*x,mean_cur(3,2)*y,mean_cur(3,3)*z);
hold off

%% Plot em
for i = 1:size(dataset,2)
    
    d = dataset{i};
    
    ia = d(1,:)*525.9366;
    ib = d(2,:)*340.4033;
    ic = d(3,:)*185.8066;


    ic_est = -ia-ib;
    garr4 = ia / (2*ib);
    garr5 = ib / (2*ia);

    figure(i)
    clf
    hold on
    plot(ia);
    plot(ib);
    plot(ic);
    % plot(ic_est);
    % plot(ia+ib+ic_est);
end
for(i = 1:3)
    figure(3+i)
    hold on
    plot(ones(1, length(ic) )*mean_cur(i,1));
    plot(ones(1, length(ic) )*mean_cur(i,2));
    plot(ones(1, length(ic) )*mean_cur(i,3));
    legend('ia','ib','ic','meana','meanb','meanc');
    hold off
end
for(i = 1:3)
    figure(6+i)
    plot(ones(1, length(ic) )*(mean_cur(i,1)*sols(1) + mean_cur(i,2)*sols(2) + mean_cur(i,3)*sols(3)));
    legend('ia + ib + ic');
end

%% 
% We plotted 3 lines earlier that are all on the same plane. That plane is the infinite set of
% possible weights (That's what the result sols = [0, 0, 0] was telling us - we had a system of equations 
% with an infinite number of solutions, which we knew bc all the currents must always add up to zero.]
% So we want to find the equation of that plane. 
% Planes are defined by two vectors. So we will:
% 1. Find the equations of any two of the lines (I pick the ones from
% dataset{4} and dataset{5}
% 2. Find the vectors that these equations describe.
% 3. Take the dot product of the two vectors to find the equation of the
% plane.
% 4. Pick an arbitrary point on the plane (this point corresponds to one
% of the three weights).
% 5. Solve the equation of the plane for the other two weights.
%Equations
% mean_cur(1, 1)*x + mean_cur(1, 2)*y + mean_cur(1, 3)*z = 0;
% mean_cur(2, 1)*x + mean_cur(2, 2)*y + mean_cur(2, 3)*z = 0;
%Vectors
v1 = [mean_set1(1, 1), mean_set1(1, 2), mean_set1(1,3)];
v2 = [mean_set1(2, 1), mean_set1(2, 2), mean_set1(2,3)];

normal = cross(v1, v2);
syms x y z
p = [x, y, z];
plane = dot(normal, p)


