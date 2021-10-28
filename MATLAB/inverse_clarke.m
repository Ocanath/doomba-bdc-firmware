function [ia,ib,ic] = inverse_clarke(i_alpha,i_beta)
    ia = i_alpha;
    ib = -.5*i_alpha + (sqrt(3)/2)*i_beta;
    ic = -.5*i_alpha - (sqrt(3)/2)*i_beta;
end