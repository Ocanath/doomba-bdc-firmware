% Inputs:
%   Adc data y and x. 
%   Expected input range is [-2048 to 2048], int16. This means adc data is 
%   input biased BEFORE the call to this function
%
% Outputs: 
%   atan2, fixed point. 4096*pi -> pi, -4096*pi -> -pi.
%   OUTPUT RANGE -12868 to 12868 (off by 1 possible)
function out = atan2_fixed(y,x)
    assert(isa(y,'int32') & isa(x,'int32'),'Error: inputs must be of type int32');
    
    FP_HALF_PI = 6434;  %int32(4096*pi/2);
    FP_PI = 12868;
    
    if(x == 0)
        if(y == 0)
            out = 0;
        elseif(y > 0)
            out = FP_HALF_PI; %pi/2
        else
            out = -FP_HALF_PI; %-pi/2
        end
        return;
    end
    
    if(y == 0)
        if(x > 0)
            out = 0;
        else
            out = FP_PI;    %pi
        end
        return;
    end

    % get absolute value of both
    abs_s = y;
    if(abs_s < 0)
        abs_s = -abs_s;
    end
    abs_c = x;
    if(abs_c < 0)
        abs_c = -abs_c;
    end
    
    % get min value of both
    minv = abs_c;
    maxv = abs_s;
    if(maxv < minv)
        minv = abs_s;
        maxv = abs_c;
    end
    
%     % do a fixed point division...
    % apply a fixed point scaling factor of 2^12 with a shift
    a = bitshift(minv,12)/maxv; %this is guaranteed to range from 0-4096
%                                 %as min/max logic above constrains a to
%                                 %nominally be 0-1
%                                 %2^12 is our scf for atan2, and the 
%                                 % max resolution of the encoder, so it 
%                                 %works out nicely
    
    a2 = bitshift(a*a,-12); %remove double application of scaling factor 2^12 with a rightshift
    a3 = bitshift(a2*a,-12);
    a4 = bitshift(a3*a,-12);
    a5 = bitshift(a4*a,-12);
    %best, most multiplies, 4th order
%     c = int32([580, -1406, -66, 4112, -1]); %this also has a factor of 4096
    c = int32([4637      -11250        -532       32894          -5]);
    r = a4*c(1) + a3*c(2) + a2*c(3) + a*c(4) + c(5);
    r = bitshift(r,-3);
    
    %second best, 3rd order
%     c = int32([-247,-812,4277,-9]);
%     r = a3*c(1) + a2*c(2) + a*c(3) + c(4);
    
    %third best, 2nd order. Fastest
%     c = int32([-1182, 4425, -21]);
%     r = a2*c(1) + a*c(2) + c(3);

    %fifth order. slower, more accurate than 4th. you get the picture
%     c = int32([-1778, 9083, -15201, 950, 32682, 2]); %this is fifth order poly
% 	r = a5*c(1) + a4*c(2) + a3*c(3) + a2*c(4) + a*c(5) + c(6);
%     r = bitshift(r,-3); %need to shift 15 for these coefficients
    
%     r = r/12867;    % 1/(4096*pi). this is sort of like res(0-1) = res*4096/pi,
                    % except that we have 2 factors of 4096 already present
                    % in r from its presence in a2, a3 and a4 as well as
                    % the constants in c. So then it'
                    % (res*4096*4096)/(4096*pi), which is the same as
                    % res*4096/pi if res is from 0-1.
                    %
                    %If we just divide by 4096 (i.e. bitshift), then that
                    %changes what value pi is equal to from the calculation
                    %result. just 1/4096 makes pi equal to (4096*pi)
                    
    r = bitshift(r,-12);    %remove one of 4096, but leave one for scaling. Using a shift here means the output format is a multiple of pi
    
    %fp_coef from fixed_point_foc
    if(abs_s > abs_c)
        r = FP_HALF_PI - r;
    end
    if(x < 0)
        r = FP_PI - r;
    end
    if(y < 0)
        r = -r;
    end
    out = r;
end