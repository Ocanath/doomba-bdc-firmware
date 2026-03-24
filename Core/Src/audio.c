#include "audio.h"

static uint32_t prev_us = 0;
/*
 * Inputs:
 * a: pointer to audio streamer control structure. Contains playback sampling rate
 * t_us: time in microseconds. This overflows - program relies on unsigned subtraction for proper delta
 * */
int32_t audio_stream(audio_renderer_t * a, uint32_t t_us)
{
	if(a == NULL)
	{
		return 0;
	}

	if(t_us - prev_us >= a->retransmission_us)
	{
		prev_us = t_us;
		if(a->buffer_pos + 1 < (sizeof(a->recv_buffer)/sizeof(int16_t)))
		{
			a->buffer_pos++;
			return a->recv_buffer[a->buffer_pos];
		}
		else
		{
			return 0;
		}
	}
	else
	{
		if(a->buffer_pos >= (sizeof(a->recv_buffer)/sizeof(int16_t)))
		{
			return 0;
		}
		return a->recv_buffer[a->buffer_pos];
	}
}
