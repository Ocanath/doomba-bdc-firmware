% This doesn't work well for fixed point.
% Unwraps 12bit angle (from -pi to pi) into
% a 32 bit angle (-2^31-1 to 2^31-1). 
%
% This is only suitable for systems where the robotic
% *input* joint rotates less than 500,000 times (i.e. pre-gearbox).
% Otherwise it will eventually wrap
%
function [out, prev_out, overflow_cnt] = unwrap32_fixed(in, prev_out, overflow_cnt, HALF_OVERFLOW_VAL)
    assert(isa(HALF_OVERFLOW_VAL,'int32') & isa(in,'int32') & isa(prev_out,'int32'),'Error:input type must be int32');
    PI_12B = int32(pi*4096);
%     HALF_OVERFLOW_VAL = 6*int32(pi*4096);
    OVERFLOW_VAL = 2*HALF_OVERFLOW_VAL;
    
    
    wrap_dif = mod( (in-prev_out) + PI_12B, PI_12B*2)-PI_12B;
    unwrapped = prev_out + wrap_dif;
    if(unwrapped >= HALF_OVERFLOW_VAL)
        unwrapped = unwrapped - OVERFLOW_VAL;
        overflow_cnt = overflow_cnt + 1;
    elseif(unwrapped <= -HALF_OVERFLOW_VAL)
        overflow_cnt = overflow_cnt - 1;
        unwrapped = unwrapped + OVERFLOW_VAL;
    end
    prev_out = unwrapped;
    out = unwrapped;
end