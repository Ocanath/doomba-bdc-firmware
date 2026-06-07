#include "trig_fixed.h"
#include <math.h>
#include "unity.h"

#define ONE_14B	((double)(1<<14))
#define ONE_12B ((double)(1<<12))

void test_wrap(void)
{
	int32_t a = PI_14B;
	int32_t tv = 2;
	int32_t b = wrap_2pi_q(a + tv, TWO_PI_14B);
	TEST_ASSERT_EQUAL(-PI_14B + tv, b);

	tv = TWO_PI_14B+10;
	TEST_ASSERT_EQUAL(wrap_2pi_q(tv, TWO_PI_14B), wrap_2pi_14b(tv));

	TEST_ASSERT_EQUAL(wrap_2pi_q(tv, TWO_PI_12B), wrap_2pi_12b(tv));
}

double m_abs(double in)
{
	if(in < 0)
	{
		return -in;
	}
	return in;
}

void test_sin14b(void)
{
	//strict worst-case LSB error bounded from python script (see ../Python/trig_fixed/sin_cos_coefficients_fixed.py)
	//we check every value within the valid range (-pi to pi) to guarantee
	const double q_factor = (double)(1 << 14);	//2^14 fixed for this function
	double max_error = 0;
	for(int32_t theta = -PI_14B; theta < PI_14B; theta++)
	{
		double theta_dbl = (double)theta / q_factor;
		double s_std = sin(theta_dbl)*q_factor;
		double s14 = (double)sin_14b(theta);
		double error = m_abs(s_std - s14);
		if(error > max_error)
		{
			max_error = error;
		}
		TEST_ASSERT_LESS_THAN_DOUBLE(2.95001, error);
	}
	printf("Max LSB for sin_14b: %f\n", max_error);
}

void test_cos14b(void)
{
	//strict worst-case LSB error bounded from python script (see ../Python/trig_fixed/sin_cos_coefficients_fixed.py)
	//we check every value within the valid range (-pi to pi) to guarantee
	const double q_factor = (double)(1 << 14);	//2^14 fixed for this function
	double max_error = 0;
	for(int32_t theta = -PI_14B; theta < PI_14B; theta++)
	{
		double theta_dbl = (double)theta / q_factor;
		double s_std = cos(theta_dbl)*q_factor;
		double s14 = (double)cos_14b(theta);
		double error = m_abs(s_std - s14);
		if(error > max_error)
		{
			max_error = error;
		}
		TEST_ASSERT_LESS_THAN_DOUBLE(2.95001, error);
	}
	printf("Max LSB for cos_14b: %f\n", max_error);
}

void test_sin_cos_14b(void)
{
    const double q_factor = (double)(1 << 14);
    double max_sin_err = 0, max_cos_err = 0;
    for (int32_t theta = -PI_14B; theta < PI_14B; theta++)
    {
        double theta_dbl = (double)theta / q_factor;
        int32_t s, c;
        sin_cos_14b(theta, &s, &c);

        double sin_err = m_abs(sin(theta_dbl)*q_factor - (double)s);
        double cos_err = m_abs(cos(theta_dbl)*q_factor - (double)c);
        if (sin_err > max_sin_err) max_sin_err = sin_err;
        if (cos_err > max_cos_err) max_cos_err = cos_err;
        TEST_ASSERT_LESS_THAN_DOUBLE(2.95001, sin_err);
        TEST_ASSERT_LESS_THAN_DOUBLE(3.09001, cos_err);
    }
    printf("Max LSB sin_cos_14b: sin=%f  cos=%f\n", max_sin_err, max_cos_err);
}

void test_sin12b(void)
{
	float val = 1.24515616f;
	int32_t val_12b = (int32_t)(val*(float)(1 << 12));
	int32_t res = sin_12b(wrap_2pi_q(val_12b, TWO_PI_12B));
	float res_sc = ((float)res)/((float)(1 << 12));
	TEST_ASSERT_FLOAT_WITHIN(0.0001f, sin(val), res_sc);
}


void test_atan(void)
{
	double xarr[] = {0.1, -0.151, -0.1231, 0.311};
	double yarr[] = {.1235, 0.151, -0.1231, -0.311};	//these should be normalized or smaller - otherwise will have significant errors
	TEST_ASSERT_EQUAL(sizeof(xarr),sizeof(yarr));

	for(int i = 0; i < sizeof(xarr)/sizeof(double); i++)
	{
		double x = xarr[i];
		double y = yarr[i];

		double th = atan2(y, x);
		int32_t x14 = (int32_t)(x*ONE_14B);
		int32_t y14 = (int32_t)(y*ONE_14B);
		int32_t res = atan2_14b(y14, x14);
		double resdbl = ((double)res) / ONE_14B;
		TEST_ASSERT_FLOAT_WITHIN(0.001, th, resdbl);

		int32_t x12 = (int32_t)(x*ONE_12B);
		int32_t y12 = (int32_t)(y*ONE_12B);
		res = atan2_12b(y12, x12);
		resdbl = ((double)res) / ONE_12B;
		TEST_ASSERT_FLOAT_WITHIN(0.001, th, resdbl);

	}
}


void test_unwrap(void)
{
	unwrap_state_t st = {
		.ovfl_cnt = 0,
		.prev_theta_wrapped = 0,
		.unwrapped_angle = 0
	};
	st.unwrapped_angle = 0x7FFFFFFF;
	int32_t theta = 1;
	unwrap_angle_32b_overflow(theta, TWO_PI_14B, &st);
	TEST_ASSERT_EQUAL(1, st.ovfl_cnt);

	theta = 0;
	unwrap_angle_32b_overflow(theta, TWO_PI_14B, &st);
	TEST_ASSERT_EQUAL(0, st.ovfl_cnt);

	st.unwrapped_angle = -0x7FFFFFFF;	//because it is signed, 0x80000000 is max negative, and 0x7FFFFFFF is max positive. 
	theta = -1;
	unwrap_angle_32b_overflow(theta, TWO_PI_14B, &st);	//first decrement gets us to max negative
	TEST_ASSERT_EQUAL(0, st.ovfl_cnt);
	theta = -2;								//we have to decrease theta further to decrement to the overflow case
	unwrap_angle_32b_overflow(theta, TWO_PI_14B, &st);	
	TEST_ASSERT_EQUAL(-1, st.ovfl_cnt);
	

	st.prev_theta_wrapped = 0;
	st.unwrapped_angle = 0;
	st.ovfl_cnt = 0;
	int32_t i;
	for(i = 20; i < TWO_PI_14B*6; i++)
	{
		theta = wrap_2pi_14b(i);
		unwrap_angle_32b_overflow(theta, TWO_PI_14B, &st);
	}
	TEST_ASSERT_NOT_EQUAL(theta, st.unwrapped_angle);
	TEST_ASSERT_EQUAL(i-1, st.unwrapped_angle);

	for(; i > -TWO_PI_14B*6; i-= 1000)
	{
		theta = wrap_2pi_14b(i);
		unwrap_angle_32b_overflow(theta, TWO_PI_14B, &st);
	}
	TEST_ASSERT_NOT_EQUAL(theta, st.unwrapped_angle);
	TEST_ASSERT_EQUAL(i+1000, st.unwrapped_angle);


}

void test_unwrap_recovery(void)
{
	unwrap_state_t st = {
		.ovfl_cnt = 0,
		.prev_theta_wrapped = 0,
		.unwrapped_angle = 0
	};
	int32_t angle_start = 0x7FFFFFFF;
	st.unwrapped_angle = angle_start;
	st.prev_theta_wrapped = 10;
	int32_t offset = -20;
	int32_t angle_offsetted = unwrap_angle_32b_overflow(20, TWO_PI_14B, &st) - offset;

	int64_t angle_recovered = ((int64_t)st.ovfl_cnt)*(0x100000000LL) + (int64_t)angle_offsetted;
	int64_t angle_recovered_2 = ((int64_t)st.ovfl_cnt)*(0x100000000LL) + (int64_t)st.unwrapped_angle - (int64_t)offset;

	TEST_ASSERT_EQUAL_INT64(2147483677, angle_recovered);
	TEST_ASSERT_EQUAL_INT64(2147483677, angle_recovered_2);
}


