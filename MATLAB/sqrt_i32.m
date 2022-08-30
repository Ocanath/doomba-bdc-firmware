%// sqrt_i32 computes the squrare root of a 32bit integer and returns
%// a 32bit integer value. It requires that v is positive.
function q = sqrt_i32(v) 
    assert(isa(v,'int32'));
    
	b = bitshift(uint32(1), 30);
    q = uint32(0);
    r = uint32(v);
    while (b > r)
        b = bitshift(b, -2);    % 	b >>= 2;
    end
    while (b > 0) 
		t = uint32(q + b);
        q = bitshift(q, -1);    %	q >>= 1;
        if (r >= t) 
			r = r - t; %r -= t;
			q = q + b; %q += b;
        end
		b = bitshift(b, -2);%b >>= 2;
    end
end