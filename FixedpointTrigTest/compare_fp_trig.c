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

int main(void)
{

    printf("t, err12b, err14b\n");
    /*Test of sin*/
    double increment = .001;
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

        printf("%f, %f, %f\n", t, e12, e14);
        //printf("%f, %f, %f\r\n", ref_sin, ds12, ds14);
    }
    
    // printf("Max error 12b: %fe-3\r\n", max_error_12b*1000.);
    // printf("Max error 14b: %fe-3\r\n", max_error_14b*1000.);

}