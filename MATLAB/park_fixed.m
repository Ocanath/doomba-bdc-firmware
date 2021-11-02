function [iq, id] = park_fixed(alpha,beta,sin_theta,cos_theta)
    assert(isa(alpha,'int32') & isa(beta,'int32') & isa(sin_theta,'int32') & isa(cos_theta,'int32') ,'Error:input type must be int32');
    id = bitshift((alpha.*cos_theta + beta.*sin_theta), -12);
	iq = bitshift((-alpha.*sin_theta + beta.*cos_theta), -12);
end