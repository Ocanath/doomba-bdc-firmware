function out = wrap(in,k)
    half_k = k/2;
    out = mod(in+half_k,k)-half_k;
end