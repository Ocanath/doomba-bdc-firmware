#include "audio.h"
#include "unity.h"
#include <math.h>

void test_audio_stream_nulptr(void)
{
	int32_t v = audio_stream(NULL);
	TEST_ASSERT_EQUAL(0, v);
}

void test_audio_stream(void)
{
	audio_renderer_t a = {};
	for(int i = 0; i < sizeof(a.recv_buffer)/sizeof(int16_t); i++)
	{
		a.recv_buffer[i] = i+1;	
		TEST_ASSERT_NOT_EQUAL(0, a.recv_buffer[i]);	//must be nonzero or test is confused
	}

	//overrun test - confirm nonzero return for overrun pointer
	a.buffer_pos = sizeof(a.recv_buffer) + 10;
	int32_t v = audio_stream(&a);
	TEST_ASSERT_EQUAL(0, v);

	//TODO: rework with mocked timer
	a.buffer_pos = 0;
	v = audio_stream(&a);
	TEST_ASSERT_EQUAL(1, v);
}
