#ifndef MOTOR_H
#define MOTOR_H

#include "dartt_mctl_params.h"
#include <vector>
#include "dartt_sync.h"
#include "serial_callbacks.h"
#include "serial.h"

class Motor
{
public:
	dartt_mctl_params_t dp_ctl;
	dartt_mctl_params_t dp_periph;
	dartt_sync_t ds;
	unsigned char tx_buf_mem[SERIAL_BUFFER_SIZE];
	unsigned char rx_buf_mem[SERIAL_BUFFER_SIZE];

	Motor(unsigned char addr, Serial * ser);

	Motor(const Motor&) = delete;
	Motor& operator=(const Motor&) = delete;

	float q;	//radians
	float iq;	//unconverted adc units for now
	float id;	//unconverted adc units (for now)
	float qdot;	//radians per second

	float qd;	//also radians. wrapped on target.
	float qdset;

	int read_motion_data(void);
	int read_time_data(void);	//get time in ms from device
	int write_command_word(int32_t command_word);
	int write_pctl_data(void);
	int rezero(void);	//set current position to zero on the motor
private:
	dartt_mem_t read_slice;
	dartt_mem_t ms_slice;	//slice of backing store corresponding to time in ms
	dartt_mem_t write_slice;

};

#endif