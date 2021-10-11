% This doesn't work well for fixed point.
% Unwraps 12bit angle (from -pi to pi) into
% a 32 bit angle (-2^31-1 to 2^31-1). 
%
% This is only suitable for systems where the robotic
% *input* joint rotates less than 500,000 times (i.e. pre-gearbox).
% Otherwise it will eventually wrap
%
function out = unwrap32_fixed(in, prev_out)

end