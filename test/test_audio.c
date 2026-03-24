#include "audio.h"
#include "unity.h"
#include <math.h>

void test_audio_stream_nulptr(void)
{
	int32_t v = audio_stream(NULL, 0);
	TEST_ASSERT_EQUAL(0, v);
}

void test_audio_stream_overrun_overflow(void)
{
	audio_renderer_t a = {};
	a.retransmission_us = 25;	//40khz, standard 
	for(int i = 0; i < sizeof(a.recv_buffer)/sizeof(int16_t); i++)
	{
		a.recv_buffer[i] = i+1;	
		TEST_ASSERT_NOT_EQUAL(0, a.recv_buffer[i]);	//must be nonzero or test is confused
	}

	//overrun test - confirm nonzero return for overrun pointer
	a.buffer_pos = sizeof(a.recv_buffer) + 10;
	int32_t v = audio_stream(&a, 0);
	TEST_ASSERT_EQUAL(0, v);

	//sampling - confirm that a new value occurs DIRECTLY on the microsecond edge transition 
	a.buffer_pos = 0;
	v = audio_stream(&a, 0);
	TEST_ASSERT_EQUAL(1, v);
	v = audio_stream(&a, 1000);
	TEST_ASSERT_EQUAL(2, v);
	v = audio_stream(&a, 1023);
	TEST_ASSERT_EQUAL(2, v);
	v = audio_stream(&a, 1024);
	TEST_ASSERT_EQUAL(2, v);
	v = audio_stream(&a, 1025);
	TEST_ASSERT_EQUAL(3, v);


	//integer overflow test
	int32_t v1 = audio_stream(&a, 0xFFFFFFFF-10);
	int32_t v2 = audio_stream(&a, 0xFFFFFFFF-10);
	TEST_ASSERT_EQUAL(4, v1);
	TEST_ASSERT_EQUAL(4, v2);
	v2 = audio_stream(&a, 10);
	TEST_ASSERT_EQUAL(4, v2);
	v2 = audio_stream(&a, 14);
	TEST_ASSERT_EQUAL(5, v2);
	v2 = audio_stream(&a, 16);
	TEST_ASSERT_EQUAL(5, v2);
	
}
