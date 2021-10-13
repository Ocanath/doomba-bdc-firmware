#ifndef  FOC_FIXED_H
#define FOC_FIXED_H
#include "trig_fixed.h"

void svm_fixed(int32_t alpha, int32_t beta, int32_t period_cnt, int32_t* tA, int32_t* tB, int32_t* tC);
void inverse_park_fixed(int32_t iq, int32_t id, int32_t  sth, int32_t cth, int32_t* ialpha, int32_t* ibeta);

#endif // ! FOC_FIXED_H
