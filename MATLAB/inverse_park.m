%Inputs: 
%    iq, id, desired
%    sth, cth measured
%    IMPORTANT NOTE:    
%        Fixed point range of iq and id is not full. If we use 2^12, the
%        true range will be (sqrt(3)/2)*2^12, or +/-3547
%       
%   Outputs:
%       ialpha,ibeta in whichever scale iq and id were passed
%       
%   
function [ialpha,ibeta] = inverse_park(iq,id,sth,cth)
    ialpha = id*cth - iq*sth;
    ibeta = id*sth + iq*cth;
end