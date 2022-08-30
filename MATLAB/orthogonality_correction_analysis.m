%%
%Verification of the method in the datasheet. The math still confuses me a
%bit but it clearly works

%First establish the phase errors for the sin-cos terms
ec = 5*pi/180;    %
es = -5*pi/180;
% ec = 152/4096;
% es = 0;

%Generate reference (true mechanical) and phase-error 'unit' vector list
t = -pi-1:1/1000:pi+1;
vref = [cos(t)',sin(t)'];   %true mechanical vectors
verr = [cos(t+ec)', sin(t+es)'];    %erroneous vectors
angref = atan2(vref(:,2),vref(:,1));

%Take error sin and error cos. Back out the angles from the erorr vectors
%with atan2. 
angerr = atan2(verr(:,2),verr(:,1));

%from the atan2 error angle vector, compute the locations closest to
%sin-cos 45. This is the same as using a motor to track the 45 and 135
%degree encoder positions using atan2 encoder sig as a control reference
[~, idx45] =  min(abs(angerr-45*pi/180)); 
[~, idx135] = min(abs(angerr-135*pi/180));
v45 = verr(idx45,:);
v135 = verr(idx135,:);
%get the magnitudes of these vectors
M45 = sqrt(dot(v45,v45));
M135 = sqrt(dot(v135,v135));

epsilon = 2*atan2(M135-M45, M135+M45);
fprintf("eps = %f degrees\n", epsilon*180/pi);

y3 = ( verr(:,2) - verr(:,1)*sin(-epsilon) ) / cos(-epsilon);
alpha = atan2(y3 , verr(:,1))';

vcorrected = [cos(alpha)', sin(alpha)'];
angcorrected = atan2(vcorrected(:,2),vcorrected(:,1));

figure(2)
clf
axis equal
hold on
plot([0,cos(45*pi/180)],[0,sin(45*pi/180)]);
plot([0,v45(1)],[0,v45(2)]);
plot([0,cos(135*pi/180)],[0,sin(135*pi/180)]);
plot([0,v135(1)],[0,v135(2)]);
hold off
xlim([-1.1,1.1])
ylim([-1.1,1.1])
legend('ref45','err45','ref135','err135','location','southeast');

% figure(3)
% clf
% axis equal
% hold on
% plot(vref(:,1) , vref(:,2));
% plot(verr(:,1) , verr(:,2));
% plot(vcorrected(:,1) , vcorrected(:,2));    
% % plot(cos(angerr),sin(angerr));
% plot(cos(atan2(verr(:,2),verr(:,1))), sin(atan2(verr(:,2),verr(:,1))))
% hold off
% xlim([-1.1,1.1]);
% ylim([-1.1,1.1]);
% legend('true','bad','corrected (shifted for visibility)', 'atan2-ed error');


figure(4)
clf
hold on
plot(cos(angerr))
plot(sin(angerr))
plot(cos(angref))
plot(sin(angref))
% plot(
hold off
legend('err cos','err sin', 'true cos', 'true sin');

figure(5)
clf
hold on
plot(angref);
plot(angerr)
plot(mod(angcorrected - ec + pi, 2*pi)-pi)
% wc = mod(angcorrected - epsilon + pi, 2*pi)-pi;
% plot(wc)
hold off
legend('ref','err','corrected','location','southeast');

%%

sth = sin(t+ec);
cth = cos(t+es);

figure(1)
clf
hold on
plot(sin(t))
plot(cos(t))
plot(sin(t+es))
plot(cos(t+ec))
hold off

