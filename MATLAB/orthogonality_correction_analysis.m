%%
%Verification of the method in the datasheet. The math still confuses me a
%bit but it clearly works

%First establish the phase errors for the sin-cos terms
ec = -2.4*pi/180;    %
es = 1*pi/180;

%Generate reference (true mechanical) and phase-error 'unit' vector list
t = -pi-1:1/1000:pi+1;
vref = [cos(t)',sin(t)'];
verr = [cos(t+ec)', sin(t+es)'];

[~, idx45] =  min(abs(atan2(verr(:,2),verr(:,1))-45*pi/180));
[~, idx135] = min(abs(atan2(verr(:,2),verr(:,1))-135*pi/180));

v45 = verr(idx45,:);
v135 = verr(idx135,:);
M45 = sqrt(dot(v45,v45));
M135 = sqrt(dot(v135,v135));

epsilon = 2*atan2(M135-M45, M135+M45);
fprintf("eps = %f degrees\n", epsilon*180/pi);

y3 = ( verr(:,2) - verr(:,1)*sin(-epsilon) ) / cos(-epsilon);
alpha = atan2(y3 , verr(:,1))';

vcorrected = [cos(alpha)', sin(alpha)'];


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

figure(3)
clf
axis equal
hold on
plot(vref(:,1) , vref(:,2));
plot(verr(:,1) , verr(:,2));
plot(vcorrected(:,1) , vcorrected(:,2));    
hold off
xlim([-1.1,1.1]);
ylim([-1.1,1.1]);
legend('true','bad','corrected (shifted for visibility)');

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

