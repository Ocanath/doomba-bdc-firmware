/*
 * calibration_fixed.c
 *
 *  Created on: Oct 26, 2021
 *      Author: Ocanath Robotman
 */
#include <FDCAN.h>
#include "foc_fixed.h"
#include "init.h"
#include "settings.h"
#include "m_uart.h"
#include "calibration_fixed.h"
#include "adc.h"
#include "mag-encoder.h"
#include "foc_commutation.h"

/*TODO: change doubles to int32's and int64's for lower memory consumption.*/

/*Human plot analysis (atm) for determining the best align offset thru a linear sweep of all align offsets.
 *
 * Provides, at minimum, a reference point for the superior auto/gradient descent method which doesn't cause it to fall
 * into a local minima.
 *
 * */
void dowse_for_align_offset(void)
{
	int32_t max_abs_dq = 0;
//	volatile int32_t aoff_of_dqmax = 0;
	uint32_t start_tick = HAL_GetTick();
	uint32_t uart_tx_ts = 0;
	int32_t prev_gl_theta_rem_m = 0;
	int32_t dif = 0;
	int32_t avg_v_ts = 0;
	while(1)
	{
		uint32_t tick = (HAL_GetTick()-start_tick);

		uint32_t tbasis = tick/15;
		fds_mp->align_offset_fixed = wrap_2pi_12b(tbasis);

		sinusoidal_fixed(500,0);
		get_dtheta();

		int32_t avq = abs_i32(gl_dp.dtheta_fixedpoint_rad_p_sec);
		if(avq > max_abs_dq)
		{
			max_abs_dq = avq;
//			aoff_of_dqmax = fds_mp->align_offset_fixed;
		}

		if(tick >= avg_v_ts)
		{
			avg_v_ts = tick + 50;
			dif = (gl_dp.theta_rem_m - prev_gl_theta_rem_m);
			prev_gl_theta_rem_m = gl_dp.theta_rem_m;
		}

		if(tick >= uart_tx_ts)
		{
			uart_tx_ts = tick + 10;

			int32_fmt_t fmt;
			int bidx = 0;
			uint8_t buf[8*sizeof(int32_t)] = {0};

			fmt.v = dif;
			buffer_data(fmt.d, sizeof(int32_t), buf, &bidx);
			fmt.v = fds_mp->align_offset_fixed;
			buffer_data(fmt.d, sizeof(int32_t), buf, &bidx);
//			fmt.v = fds_mp->align_offset_fixed;
//			buffer_data(fmt.d, sizeof(int16_t), buf, &bidx);

			m_uart_tx_start((uart_it_t*)&m_huart2, buf, 2*sizeof(int32_t));	//HAL_UART_Transmit_IT(&huart2,buf,4*sizeof(float));
		}
	}
	TIMER_UPDATE_DUTY(500,500,500);
}


int32_t abs_i32(int32_t in)
{
	if(in < 0)
		return -in;
	else
		return in;
}

int64_t abs64(int64_t in)
{
	if(in > 0)
		return in;
	else
		return -in;
}

int32_t run_v_fixed(int32_t v, uint32_t run_time, int64_t * dist)
{
	for(uint32_t stop_ts = HAL_GetTick() + 100; HAL_GetTick()<stop_ts;)	//initial run for SS
		sinusoidal_fixed(v,0);	//todo: replace this with a voltage control mode call
	int64_t run_start = gl_dp.theta_rotations*PI_12B + gl_dp.theta_rem_m;
	for(uint32_t stop_ts = HAL_GetTick() + run_time; HAL_GetTick()<stop_ts;)	//the run
		sinusoidal_fixed(v,0);
	int64_t run_end = gl_dp.theta_rotations*TWO_PI_12B + gl_dp.theta_rem_m;
	TIMER_UPDATE_DUTY(0,0,0);
	HAL_Delay(50);
	*dist = run_end-run_start;
	int64_t run_speed = (( (*dist) << 16)/run_time);
	if(run_speed >= 0)
		return run_speed;
	else
		return -run_speed;
}

/*Working!!!*/
int32_t get_elec_conversion_ratio(void)
{
	int32_t theta = 0;
	int32_t theta_elec = wrap_2pi_12b(theta);
	int32_t sth = sin_12b(theta_elec);
	int32_t cth = cos_12b(theta_elec);
	int32_t alpha,beta,tA,tB,tC;
	inverse_park_fixed(300,0,sth,cth,&alpha,&beta, 12);
	svm_fixed(alpha,beta,TIM1->ARR,&tA,&tB,&tC);
	TIMER_UPDATE_DUTY(tA,tB,tC);
	HAL_Delay(100);

	int32_t theta_m_start = theta_rel_12b();
	uint8_t one_m_rev = 0;
	int32_t prev_theta_m = 0;

	uint32_t start_tick = HAL_GetTick();
	while(one_m_rev == 0)
	{
		theta = (HAL_GetTick() - start_tick) * 50;

		theta_elec = wrap_2pi_12b(theta);
		sth = sin_12b(theta_elec);
		cth = cos_12b(theta_elec);
		inverse_park_fixed(300,0,sth,cth,&alpha,&beta, 12);
		svm_fixed(alpha,beta,TIM1->ARR,&tA,&tB,&tC);
		TIMER_UPDATE_DUTY(tA,tB,tC);

		int32_t theta_m = theta_rel_12b() - theta_m_start;	//atan2, apply align offset
		gl_dp.theta_rem_m = unwrap_angle_32b_overflow(theta_m, &prev_theta_m, TWO_PI_12B, &gl_dp.theta_rotations);

		if(gl_dp.theta_rem_m >= TWO_PI_12B+10)
		{
			one_m_rev = 1;
		}
	}
	return theta/TWO_PI_12B;
}


//void auto_calibrate_fixed(void)
//{
//	int16_t passcount = 0;
//	while(1)
//	{
//		int32_t v = MAX_VQ/2;
//		int64_t dist1,dist2;
//		int64_t run1_speed = run_v_fixed(v, 400, &dist1);
//		int64_t run2_speed = run_v_fixed(-v, 400, &dist2);
//		int8_t sign = 0;
//		if(dist1 > 0 && dist2 < 0)
//			sign = 1;
//		else if (dist1 < 0 && dist2 > 0)
//			sign = -1;
//		if(sign < 0 && abs64(dist1) > 24576 && abs64(dist2) > 24576 )
//			fds_mp->align_offset_fixed = -fds_mp->align_offset_fixed;
//
//		int64_t err = run1_speed-run2_speed;
//		int64_t u = (err* (1 << 16) )/7;	//apply a 12 bit leftshift and apply control effort reduction
//		if(sign < 0)
//			u = -u;
//		fds_mp->align_offset_fixed = (u + fds_mp->align_offset_fixed) >> 32;	//remove bitshifts applied in run_v and in the control effort calculation
//
//		if(abs64(u) < 6)	//pass criteria .0001
//		{
//			passcount++;
//			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);	//count the number of passes
//		}
//		else
//			HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,0);
//		if(passcount > 10)
//			break;	//if you've passed over 10 times, you're done!
//	}
//}

/*
 *Automatically find the encoder offset
 */
void auto_calibrate_fixed(void)
{
	TIMER_UPDATE_DUTY(500,500,500);
//	align_offset = -PI/elec_conv_ratio;
	int passcount = 0;
	while(1)
	{
		int32_t v = MAX_VQ/2;
		int64_t dist1,dist2;
		int64_t r1_sp = run_v_fixed(v, 400, &dist1);
		int64_t r2_sp = run_v_fixed(-v, 400, &dist2);

		double run1_speed = ((double)r1_sp)/((double)(1 << 16));
		double run2_speed = ((double)r2_sp)/((double)(1 << 16));
		float run_dist_1 = ((float)dist1)/4096.f;
		float run_dist_2 = ((float)dist2)/4096.f;

		float sign = 0.f;
		if(run_dist_1 > 0.f && run_dist_2 < 0.f)
			sign = 1.f;		//the direction is how it should be. positive tau means positive encoder direction
		else if (run_dist_1 < 0 && run_dist_2 > 0)
			sign = -1.f;	//the direction is flipped. positive tau means negative encoder direction

		if(sign < 0 && abs_f(run_dist_1) > 6.f && abs_f(run_dist_2) > 6.f)
			align_offset = -align_offset;		//if you're flipped and you have moved appreciably, flip the align offset and the sign should flip

		double err = run1_speed-run2_speed;	//compute base term for control effort
		double u = err/7.0;	//scale control effort down (k gain)

		align_offset += u*sign;	//adjust the align offset to converge on a value that matches the run speeds (zeros the difference)
		fds_mp->align_offset_fixed = (int32_t)(align_offset * 4096.f);
		if(abs_f(u) < 0.0001f)	//if the run speed difference is very small, it's a passable align offset
		{
			passcount++;
			HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,1);	//count the number of passes
		}
		else
			HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,0);
		if(passcount > 10)
			break;	//if you've passed over 10 times, you're done!
	}
}

void buffer_data(uint8_t * d, int size_d, uint8_t * buf, int * bidx)
{
	for(int i = 0; i < size_d; i++)
	{
		buf[*bidx] = d[i];
		*bidx = *bidx + 1;
	}
}

/*This function evaluates the fitness of our current sensors by tracking a PI loop
 * of 3 phase current waveforms. The more linear the speed of the motor, the
 * better the fitness.
 *
 * This function will plot out the results over UART
 * */
void current_sensors_fitness_test(void)
{
	fixed_PI_2_params_t g_f_a = {
			.kp = {.i32 = 9830, .radix = 3},		// (3 << 12, 10)
			.ki = {.i32 = 4194, .radix = 10},		//(1 << 12, 1000)
			.x_integral_div = 1,
			.x = 0,
			.x_sat = 700,
			.out_rshift = 12
	};
	fixed_PI_2_params_t g_f_b = {
			.kp = {.i32 = 9830, .radix = 3},		// (3 << 12, 10)
			.ki = {.i32 = 4194, .radix = 10},		//(1 << 12, 1000)			.x = 0,
			.x_integral_div = 1,
			.x = 0,
			.x_sat = 700,
			.out_rshift = 12
	};
	fixed_PI_2_params_t g_f_c = {
			.kp = {.i32 = 9830, .radix = 3},		// (3 << 12, 10)
			.ki = {.i32 = 4194, .radix = 10},		//(1 << 12, 1000)			.x = 0,
			.x_integral_div = 1,
			.x = 0,
			.x_sat = 700,
			.out_rshift = 12
	};
	int32_t iref_a, iref_b, iref_c;
	iref_a = 0;
	iref_b = 0;
	iref_c = 0;
	int32_t mag = 410*7;
	int32_t sat = 500;

	uint32_t ts = 0;
	while(1)
	{
		/*Get current in amps*4096*/
		int32_t ia = (int32_t)( (hadc1.Instance->JDR1 << 1) + 1) - current_input_offset_A;
		int32_t ib = (int32_t)( (hadc1.Instance->JDR2 << 1) + 1) - current_input_offset_B;
		int32_t ic = (int32_t)( (hadc1.Instance->JDR3 << 1) + 1) - current_input_offset_C;
		ia = (ia*curr_scf[0]) >> 13;	//from foc_fixed.c, the current conversion weights obtained with the magic method
		ib = (ib*curr_scf[1]) >> 13;
		ic = (ic*curr_scf[2]) >> 13;

		/*Track sinusoidal currents*/
		int32_t va, vb, vc;
		va = ctl_PI_fixed_2(iref_a - ia, &g_f_a);
		vb = ctl_PI_fixed_2(iref_b - ib, &g_f_b);
		vc = ctl_PI_fixed_2(iref_c - ic, &g_f_c);

		/*Our control efforts are really voltages relative to *each other*. That means we can center them about a neutral reference point without changing their effect
		 * on the plant*/
		int32_t vn = (va+vb+vc)/3;	//Create neutral point reference to center our control efforts
		va = va - vn;
		vb = vb - vn;
		vc = vc - vn;

		/*saturate control effort*/
		va = saturate(va,sat);
		vb = saturate(vb,sat);
		vc = saturate(vc,sat);
		TIMER_UPDATE_DUTY(va+562,vb+562,vc+562);

		/*Print off the voltage and current data, and generate new setpoints*/
		uint32_t tick = HAL_GetTick();
		if(tick >= ts)
		{
			ts = tick + 1;

			float ia_f = ((float)ia);
			float ib_f = ((float)ib);
			float ic_f = ((float)ic);
			floatsend_t fmt;
			int bidx = 0;
			uint8_t buf[4*sizeof(float)] = {0};
			fmt.v = ia_f;
			buffer_data(fmt.d, sizeof(float), buf, &bidx);
			fmt.v = ib_f;
			buffer_data(fmt.d, sizeof(float), buf, &bidx);
			fmt.v = ic_f;
			buffer_data(fmt.d, sizeof(float), buf, &bidx);
			fmt.v = ia_f+ib_f+ic_f;
			buffer_data(fmt.d, sizeof(float), buf, &bidx);
			m_uart_tx_start((uart_it_t*)&m_huart2, buf,4*sizeof(float));	//HAL_UART_Transmit_IT(&huart2,buf,4*sizeof(float));

			/*Generate setpoints*/
			int32_t ft = -tick*7;
			iref_a = ( mag*sin_12b(wrap_2pi_12b(ft)) >> 12);
			iref_b = ( mag*sin_12b(wrap_2pi_12b(ft + 8579)) >> 12);
			iref_c = ( mag*sin_12b(wrap_2pi_12b(ft + 17157)) >> 12);
		}
	}
}

/**/
void get_avg_cur(uint8_t phase, int32_t num_samples, int32_t voltage, double avg[NUM_CURRENT_SENSORS])
{
	int64_t sum_samples[NUM_CURRENT_SENSORS] = {0};
	int32_t mid = TIM1->ARR/2;
	int32_t va = 0;
	int32_t vb = 0;
	int32_t vc = 0;
	if(phase == 0)
		va = voltage;
	else if(phase == 1)
		vb = voltage;
	else if (phase == 2)
		vc = voltage;
	else
		return;

	uint32_t ts = HAL_GetTick() + 200;	//give it some time to hit steady state
	for(int32_t i = 0; i < num_samples;)
	{
		int32_t ia = (int32_t)( (hadc1.Instance->JDR1 << 1) + 1) - current_input_offset_A;
		int32_t ib = (int32_t)( (hadc1.Instance->JDR2 << 1) + 1) - current_input_offset_B;
		int32_t ic = (int32_t)( (hadc1.Instance->JDR3 << 1) + 1) - current_input_offset_C;
		TIMER_UPDATE_DUTY(va+mid,vb+mid,vc+mid);

		uint32_t tick = HAL_GetTick();
		if(tick >= ts)
		{
			ts = tick + 1;

			sum_samples[0] += ia;
			sum_samples[1] += ib;
			sum_samples[2] += ic;
			int bidx = 0;
			floatsend_t fmt;
			uint8_t buf[4*sizeof(int32_t)] = {0};

			fmt.v = (float)ia;
			buffer_data(fmt.d, sizeof(float), buf, &bidx);
			fmt.v = (float)ib;
			buffer_data(fmt.d, sizeof(float), buf, &bidx);
			fmt.v = (float)ic;
			buffer_data(fmt.d, sizeof(float), buf, &bidx);

			fmt.v = (float)(ia+ib+ic);
			buffer_data(fmt.d, sizeof(float), buf, &bidx);

			m_uart_tx_start((uart_it_t*)&m_huart2, buf,4*sizeof(float));	//HAL_UART_Transmit_IT(&huart2,buf,4*sizeof(float));

			i++;
		}
	}
	double num_double = (double)num_samples;
	avg[0] = ((double)sum_samples[0])/num_double;
	avg[1] = ((double)sum_samples[1])/num_double;
	avg[2] = ((double)sum_samples[2])/num_double;
	TIMER_UPDATE_DUTY(mid,mid,mid);
}

/*
	Returns vector cross product between 3 vectors A and B. Faster pass by pointer version
	ITSSS A DOUBLE CROSS
*/
void cross_double(double v_a[3], double v_b[3], double ret[3])
{
	ret[0] = -v_a[2]*v_b[1] + v_a[1]*v_b[2];
	ret[1] = v_a[2]*v_b[0] - v_a[0]*v_b[2];
	ret[2] = -v_a[1]*v_b[0] + v_a[0]*v_b[1];
}

double abs_double(double in)
{
	if(in < 0)
		return -in;
	else
		return in;
}

/*This function normalizes the current sensor calibration weights
 * for a given board by running a set of automated tests.
 *
 *This computation is performed by setting a nonzero test voltage on one phase,
 *and leaving the remaining phases as zero voltage. The current waveforms
 * for each are then averaged across some number of samples. This is repeated
 * for each phase, and for both signs of the test voltage, running a total of
 * six tests.
 *
 * Once all 6 of our average current 3-vectors are obtained, we then take
 * the cross products of different combinations of each 3 vector. These
 * cross products define a plane which is a solution space for our current
 * sensors, which are physically required by the motor to satisfy the
 * equation ia+ib+ic=0.
 *
 * The normal vector defining this plane consists
 * of the weights applied to the input adc value which will create a plane
 * that satisfies the equation ia+ib+ic=0, thus transforming our poorly
 * normalized current sensor data into correctly scaled current sensor data.
 *
 * The output of this function is normal*current_conv*2^24, where we are using 2^24 as a
 * fixed point conversion factor.
 * */
void get_cursensor_calibrations(int32_t test_voltage, int32_t w[NUM_CURRENT_SENSORS])
{
	double w1[3], w2[3], w3[3], w4[3], w5[3], w6[3];	//this uses a lot of memory!!!
	double n_avg[3] = {0.0};
	double n_cross[3] = {0.0};

	get_avg_cur(0, 3000, test_voltage, w1);
	get_avg_cur(1, 3000, test_voltage, w2);
	get_avg_cur(2, 3000, test_voltage, w3);

	get_avg_cur(0, 3000, -test_voltage, w4);
	get_avg_cur(1, 3000, -test_voltage, w5);
	get_avg_cur(2, 3000, -test_voltage, w6);

	cross_double(w1,w2,n_cross);
	for(int i = 0; i < 3; i++)
		n_avg[i] += abs_double(n_cross[i]);	//1
	cross_double(w2,w3,n_cross);
	for(int i = 0; i < 3; i++)
		n_avg[i] += abs_double(n_cross[i]); //2
	cross_double(w1,w3,n_cross);
	for(int i = 0; i < 3; i++)
		n_avg[i] += abs_double(n_cross[i]); //3

	cross_double(w4,w5,n_cross);
	for(int i = 0; i < 3; i++)
		n_avg[i] += abs_double(n_cross[i]); //4
	cross_double(w5,w6,n_cross);
	for(int i = 0; i < 3; i++)
		n_avg[i] += abs_double(n_cross[i]); //5
	cross_double(w4,w6,n_cross);
	for(int i = 0; i < 3; i++)
		n_avg[i] += abs_double(n_cross[i]); //6


	for(int i = 0; i < 3; i++)
		n_avg[i] = n_avg[i] / 6.0;
	double scf = 525.9366*n_avg[0];	//1/(525.9366) is the NOMINAL current conversion constant for THIS HARDWARE (adc -> amps)
	for(int i = 0; i < 3; i++)
	{
		n_avg[i] = (n_avg[i]*((double)(1<<24)))/scf;
		w[i] = (int32_t)(n_avg[i]);
	}

}
