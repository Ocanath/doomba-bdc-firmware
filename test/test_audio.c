#include "audio.h"
#include "unity.h"
#include <math.h>
#include <stdbool.h>

void test_audio_stream_nulptr(void)
{
	int32_t v = audio_stream(NULL, 0);
	TEST_ASSERT_EQUAL(0, v);
}

void test_stop(void)
{
	audio_renderer_t a = {};
	int32_t v = audio_stream(&a, 13411);
	TEST_ASSERT_EQUAL(0, v);
}

void test_audio_stream_overrun(void)
{
	audio_renderer_t a = {};
	audio_stream(&a, 0);	//loads up prev_us
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

	a.buffer_pos = 0;
	for(int us = 0; us < a.retransmission_us * (sizeof(a.recv_buffer)/sizeof(int16_t) + 10); us += 5)
	{
		v = audio_stream(&a, us);
		if(a.buffer_pos >= 64)
		{
			TEST_ASSERT_EQUAL(0, v);
		}
	}
	TEST_ASSERT_EQUAL(10 -1, a.buffer_pos);
	
}

void test_audio_sampling(void)
{
	audio_renderer_t a = {};
	a.retransmission_us = 25;	//40khz, standard 
	for(int i = 0; i < sizeof(a.recv_buffer)/sizeof(int16_t); i++)
	{
		a.recv_buffer[i] = i+1;	
		TEST_ASSERT_NOT_EQUAL(0, a.recv_buffer[i]);	//must be nonzero or test is confused
	}
	audio_stream(&a, 0);

	//sampling - confirm that a new value occurs DIRECTLY on the microsecond edge transition 
	a.buffer_pos = 0;
	int32_t v = audio_stream(&a, 10);
	TEST_ASSERT_EQUAL(1, v);
	v = audio_stream(&a, 1000);
	TEST_ASSERT_EQUAL(2, v);
	v = audio_stream(&a, 1023);
	TEST_ASSERT_EQUAL(2, v);
	v = audio_stream(&a, 1024);
	TEST_ASSERT_EQUAL(2, v);
	v = audio_stream(&a, 1025);
	TEST_ASSERT_EQUAL(3, v);


	//restart buffer
	a.buffer_pos = 0;
	v = audio_stream(&a, 1035);
	TEST_ASSERT_EQUAL(1, v);
	v = audio_stream(&a, 1075);
	TEST_ASSERT_EQUAL(2, v);


}

void test_audio_overflow(void)
{
	audio_renderer_t a = {};
	a.retransmission_us = 25;	//40khz, standard 
	for(int i = 0; i < sizeof(a.recv_buffer)/sizeof(int16_t); i++)
	{
		a.recv_buffer[i] = i+1;	
		TEST_ASSERT_NOT_EQUAL(0, a.recv_buffer[i]);	//must be nonzero or test is confused
	}

	//integer overflow test
	int32_t v1 = audio_stream(&a, 0xFFFFFFFF-10);
	int32_t v2 = audio_stream(&a, 0xFFFFFFFF-10);
	TEST_ASSERT_EQUAL(2, v1);
	TEST_ASSERT_EQUAL(2, v2);
	v2 = audio_stream(&a, 10);
	TEST_ASSERT_EQUAL(2, v2);
	v2 = audio_stream(&a, 14);
	TEST_ASSERT_EQUAL(3, v2);
	v2 = audio_stream(&a, 16);
	TEST_ASSERT_EQUAL(3, v2);	
}
