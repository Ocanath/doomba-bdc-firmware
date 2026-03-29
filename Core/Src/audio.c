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
	if(a->retransmission_us == 0)
	{
		prev_us = t_us;
		return 0;
	}

	if(t_us - prev_us >= a->retransmission_us)
	{
		prev_us = t_us;
		uint32_t bufsize = (sizeof(a->recv_buffer)/sizeof(int16_t));
		a->buffer_pos = (a->buffer_pos + 1) % bufsize;
		return a->recv_buffer[a->buffer_pos];
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
