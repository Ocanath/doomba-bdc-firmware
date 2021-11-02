% Function which returns the minimum number of bits
% needed to express the INTEGER TRUNCATION (i.e. rounding)
% of the input
function out = num_bits(input)
    v = round(double(input));
    bits_frac = log(v)/log(2);
    out = floor(bits_frac) + 1;
end