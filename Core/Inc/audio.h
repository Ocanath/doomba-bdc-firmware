#ifndef AUDIO_H
#define AUDIO_H
#include <stdint.h>
#include <stddef.h>

typedef struct audio_renderer_t
{
	uint32_t retransmission_us;	//time delay between adjacent samples in microseconds. sets sampling frequency
	uint32_t buffer_pos;		//read-only for writer: position in the recv buffer
	int16_t recv_buffer[64];	//look-ahead, samples. Make this an even number.
}audio_renderer_t;

int32_t audio_stream(audio_renderer_t * a, uint32_t t_us);


#endif
