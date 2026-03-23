#include "audio.h"

/**/
int32_t audio_stream(audio_renderer_t * a)
{
	if(a == NULL)
	{
		return 0;
	}
	if(a->buffer_pos >= (sizeof(a->recv_buffer)/sizeof(int16_t)))
	{
		return 0;
	}
	//TODO: wrap in a microsecond timer comparison
	//TODO: implement generic microsecond timer wrapper for mocking
	return a->recv_buffer[a->buffer_pos++];
}
