function saved_data = plot_lines(num_lines, ylowerlim, yupperlim)

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

for i = 1:num_lines
    anim_line(i) = animatedline;
end

anim_line(1) = animatedline('Color','b');

grid on;
tic;
buf_width = 6000;

linebuf = zeros(num_lines,buf_width);

ylim([ylowerlim,yupperlim]);
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
        
    pause(0.000000001);
end

fclose(s);

saved_data = linebuf;
end
