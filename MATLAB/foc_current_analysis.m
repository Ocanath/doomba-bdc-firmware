%% FOC current analysis
load('foc_current_data.mat');

dataset = {data1, data2, data3, data4, data5, data6};
%%
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
v1 = [mean_cur(1, 1), mean_cur(1, 2), mean_cur(1,3)];
v2 = [mean_cur(2, 1), mean_cur(2, 2), mean_cur(2,3)];

normal = cross(v1, v2);
syms x y z
p = [x, y, z];
plane = dot(normal, p)


