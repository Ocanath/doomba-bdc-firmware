function [dump] = get_uart_data_dump(num_inputs, type, num_elements)
    slist = serialportlist;
    baud = 230400;
    try
        s = serialport(slist(1), baud);
    catch
        disp("No serial port found");
    end
    data_idx = 1;
    dump = zeros(num_inputs,num_elements);
    while(1)
            data = read(s, num_inputs, type);
            if(isempty(data))
                break;
            end
            dump(:,data_idx) = data;
            data_idx = data_idx + 1;
            if(data_idx > num_elements)
                break;
            end
    end    
end