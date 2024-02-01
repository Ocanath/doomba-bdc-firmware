#include <math.h>
#include <stdio.h>
#include "trig_fixed.h"


double abs_v(double in)
{
    if(in < 0)
        return -in;
    else
        return in;
}

/*
 * Saturate output
 * */
int32_t saturate(int32_t v, int32_t sat)
{
	if(v > sat)
		return sat;
	if(v < -sat)
		return -sat;
	return v;
}


int main(void)
{
    // printf("t, err12b, err14b, reference, sin_12b, sin_14b\n");
    /*Test of sin*/
    double increment = .0001;
    const double pi =  3.14159265359;
    double max_error_12b = -1.;
    double max_error_14b = -1.;
    for(double t = -pi; t < pi; t+=increment)    //t increment sets 
    {
        double ref_sin = sin(t);   
        
        int32_t t12 = t*(1<<12);
        int32_t s12 = sin_12b(wrap_2pi_12b(t12));
        double ds12 = ((double)s12) / ((double)(1<<12));


        int32_t t14 = t*(1 << 14);
        int32_t s14 = sin_14b(wrap_2pi_14b(t14));
        double ds14 = ((double)s14)/( (double)(1<<14) );

        double e12 = abs_v(ref_sin - ds12);
        if(e12 > max_error_12b)
            max_error_12b = e12;


        double e14 = abs_v(ref_sin - ds14);
        if(e14 > max_error_14b)
            max_error_14b = e14;

        // printf("%f, %f, %f, %f, %d, %d\n", t, e12, e14, ref_sin, s12, s14);
    }
    
    printf("Max error 12b: %fe-3\r\n", max_error_12b*1000.);
    printf("Max error 14b: %fe-3\r\n", max_error_14b*1000.);


    max_error_12b = -1.;
    max_error_14b = -1.;
    for(double t = -pi; t < pi; t+=increment)    //t increment sets 
    {
        int32_t one_14 = 1<<14;
        int32_t one_12 = 1 << 12;

        double st = sin(t);
        double ct = cos(t);

        double at_ref = atan2(st, ct);

        int32_t s12 = (int32_t)(st*((double)(1<<11)));
        int32_t c12 = (int32_t)(ct*((double)(1<<11)));
        // int32_t at_12b = wrap_2pi_12b(atan2_12b(s12, c12) + TWO_PI_12B);
        int32_t at_12b = wrap_2pi_12b(atan2_12b(s12, c12));
        double da12 = ((double)at_12b)/ ((double)(1<<12));

        int32_t s14 = st*((double)(1<<14));
        int32_t c14 = ct*((double)(1<<14));
        int32_t at_14b = atan2_14b(s14, c14);     
        double da14 = ((double)at_14b)/ ((double)(1<<14));

        double e12 = abs_v(at_ref - da12);
        if(e12 > max_error_12b)
            max_error_12b = e12;

        if(e12 > 0.1)
        {
            printf("e12 = %f, t = %f, s12=%d, c12=%d, at_12b = %d, da12 = %f,  s14 = %d, c14 = %d\n", e12, t,  s12, c12, at_12b, da12,  s14, c14);
        }

        double e14 = abs_v(at_ref - da14);
        if(e14 > max_error_14b)
            max_error_14b = e14;


        // printf()

    }

    printf("Max error 12b: %fe-3\r\n", max_error_12b*1000.);
    printf("Max error 14b: %fe-3\r\n", max_error_14b*1000.);

    printf("res1=  %f\n", ((double)( atan2_12b(0, -2047)  ) ) / ((double)(1<<12))  );
    printf("res2=  %f\n", ((double)( atan2_14b(0, -2047)  ) ) / ((double)(1<<14))  );
}