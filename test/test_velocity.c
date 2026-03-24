#include <math.h>
#include "foc_velocity.h"
#include "dartt_mctl_params.h"
#include "trig_fixed.h"
#include "unity.h"
#include <stddef.h>
#include <stdbool.h>

/*
 * gl_v_uS_conv is normally defined in FDCAN.c (not compiled in this test build).
 * Value: 1000000 >> (14 - DEFAULT_V_REPORT_RADIX) = 1000000 >> 10 = 976
 */
int32_t gl_v_uS_conv = 1000000 >> (14 - 4);

void setUp(void)
{
	gl_dp.theta_rem_m = 0;
	gl_dp.dtheta_fixedpoint_rad_p_sec = 0;
	gl_dtheta_prev_theta = 0;
}

void tearDown(void) {}

/*
 * Test 1 (cold start): one full revolution in 1 second.
 * prev_us_dtheta = 0 at program start, dif = 1,000,000 >= 1000 -> executes.
 * Expected: (TWO_PI_14B * 976) / 1,000,000 = (102944 * 976) / 1000000 = 100
 * True velocity: 2pi rad/s * 2^4 = 100.53 -> truncated to 100.
 * After this test: prev_us_dtheta = 1,000,000.
 */
void test_dtheta_cold_start_formula(void)
{
	get_dtheta(0);
	gl_dp.theta_rem_m = PI_14B/2;
	get_dtheta(250000);
	TEST_ASSERT_EQUAL(100, gl_dp.dtheta_fixedpoint_rad_p_sec);
}

/*
 * Test 2: elapsed time below minimum period -> output must not update.
 * prev_us_dtheta = 1,000,000 (from test 1).
 * t_us = 1,000,500 -> dif = 500 < 1000 -> guard fires, no update.
 * After this test: prev_us_dtheta still 1,000,000.
 */
void test_dtheta_minimum_period_guard(void)
{
	get_dtheta(250000);
	gl_dp.theta_rem_m = PI_14B/2;
	get_dtheta(250125);
	TEST_ASSERT_EQUAL(0, gl_dp.dtheta_fixedpoint_rad_p_sec);
}

/*
 * Test 3: backward rotation (negative dtheta).
 * prev_us_dtheta = 1,000,000 (test 2 did not update it).
 * t_us = 2,000,000 -> dif = 1,000,000 >= 1000 -> executes.
 * dtheta = 0 - TWO_PI_14B = -102944
 * Expected: (-102944 * 976) / 2,000,000 = -50
 * True velocity: -2pi rad/s * 2^4 / 2 = -50.26 -> truncated to -50.
 */
void test_dtheta_negative_velocity(void)
{
	get_dtheta(0);
	gl_dtheta_prev_theta = PI_14B/2;
	gl_dp.theta_rem_m = 0;
	get_dtheta(500000);
	TEST_ASSERT_EQUAL(-50, gl_dp.dtheta_fixedpoint_rad_p_sec);
}
#define F_PI (3.14159265359f*16384.f)

void test_continuous_velocity(void)
{
	gl_dtheta_prev_theta = 0;
	gl_dp.theta_rem_m = 0;
	//true velocity: 6.25 rad/sec
	//represented velocity: 6.25*16 = 100
	//encoder tick per microsecond velocity: 6.25 * (1sec/1e6us) * (2^14enc/rad)
	double virtual_encoder_pos = 0;
	for(int us = 1000; us < 20000000; us+= 10)
	{
		virtual_encoder_pos+=1.024f;
		gl_dp.theta_rem_m = (int32_t)(virtual_encoder_pos);
		get_dtheta(us);
		if(us > 2000)
		{
			TEST_ASSERT_INT_WITHIN(1, 100, gl_dp.dtheta_fixedpoint_rad_p_sec);
		}
		
		// if(gl_dp.dtheta_fixedpoint_rad_p_sec == 98)
		// {
		// 	printf("v = %f, %d, %d\n", virtual_encoder_pos*(180.f/F_PI), gl_dp.theta_rem_m, gl_dp.dtheta_fixedpoint_rad_p_sec);
		// }
		
	}
}

/*
Use unwrap/overflow function to test compatibility with dtheta
confirms that when theta_rem_m overflows courtesy of unwrap, dtheta
still returns the proper value.
*/
void test_velocity_overflow(void)
{
	get_dtheta(0);
	gl_dp.theta_rem_m = -256;
	get_dtheta(1000);
	gl_dp.theta_rem_m = 256;
	get_dtheta(2000);
	TEST_ASSERT_EQUAL(499, gl_dp.dtheta_fixedpoint_rad_p_sec);

	get_dtheta(0);
	gl_dp.theta_rem_m = 0x7FFFFFFF - 256;
	get_dtheta(1000);
	unwrap_state_t st = {
		.ovfl_cnt = 0,
		.prev_theta_wrapped = 0,
		.unwrapped_angle = 0x7FFFFFFF - 256
	};
 	gl_dp.theta_rem_m =	unwrap_angle_32b_overflow(512, TWO_PI_14B, &st);
	get_dtheta(2000);
	TEST_ASSERT_EQUAL(499, gl_dp.dtheta_fixedpoint_rad_p_sec);
	TEST_ASSERT_EQUAL(1, st.ovfl_cnt);
	
}