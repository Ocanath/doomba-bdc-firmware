%%
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

port = 'COM7';
s=serial(port);
set(s,'BaudRate',921600);
fopen(s);


figure(1);
H = uicontrol();

num_lines = 2;
for i = 1:num_lines
    anim_line(i) = animatedline;
end

anim_line(1) = animatedline('Color','b');

grid on;
tic;
buf_width = 6000;

linebuf = zeros(num_lines,buf_width);

ylim([-50,10]);
% xlim([-1,1]);
xadv = 0;
while(ishandle(H))
    %get new value
    t = toc; %time in seconds
%     v = sin(t);
%     fprintf('%f\r\n',t);

    data = fread(s,num_lines,'single');

    set(gcf,'Renderer','OpenGL');
    for i = 1:num_lines
        
        %buffer new value. may be built into line structure
        linebuf(i,2:end) = linebuf(i, 1:end-1);
        linebuf(i,1) = data(i);
        
        clearpoints(anim_line(i));
        addpoints(anim_line(i), 1:buf_width, linebuf(i,:));
        
    end
    drawnow limitrate     
        
    pause(0.00001);
end

fclose(s);

%%
diff_offset = linebuf(1,:)-linebuf(2,:);
figure(2)
plot(linebuf(1,:));
hold on
plot(linebuf(2,:));
plot(diff_offset/3.5);
hold off

%%
figure(3)
plot(diff_offset/3.5)
mean(diff_offset(547:3000)/3.5)


%%
close all
figure(1)
hold on
plot(saved_data(1,:));
% normalized_data = (saved_data(2,:)/max(saved_data(2,:))) - 1;
% plot(normalized_data);
% maxline = zeros(1,length(saved_data(2,:)));
% maxline(1,:) = max(normalized_data);
% plot(maxline);
plot(saved_data(2,:));
hold off
