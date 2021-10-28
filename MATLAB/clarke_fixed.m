function [ialpha,ibeta] = clarke_fixed(ia,ib,ic)
    assert(isa(ia,'int32') & isa(ib,'int32') & isa(ic,'int32'),'Error: input types must be int32');
    TWO_BY_SQRT_3_12B	= int32(4730);
    ONE_BY_SQRT_3_12B	= int32(2365);
    ialpha = ia;
    ibeta = bitshift((ONE_BY_SQRT_3_12B*ia + TWO_BY_SQRT_3_12B*ib) , -12);
end