// Example for STM32F4DISCOVERY development board

#include "gpio/gpio.hpp"
#include "drv/enc/enc.hpp"
#include "systick/systick.hpp"
#include "FreeRTOS.h"
#include "task.h"

using namespace hal;
using namespace drv;

static void enc_task(void *pvParameters)
{
	enc *_enc = (enc *)pvParameters;
	while(1)
	{
		_enc->poll();
		vTaskDelay(1);
	}
}

static void enc_cb(enc *enc, int8_t diff, void *ctx)
{
	int32_t *enc_counter = (int32_t *)ctx;
	
	// diff is +1 or -1
	if(diff < 0)
		enc_counter--;
	else
		enc_counter++;
}

int main(void)
{
	systick::init();
	static gpio enc_a(0, 7, gpio::mode::DI, 1);
	static gpio enc_b(0, 8, gpio::mode::DI, 1);
	
	static enc _enc(enc_a, enc_b);
	static int32_t enc_counter;
	_enc.cb(enc_cb, &enc_counter);
	
	xTaskCreate(enc_task, "di", configMINIMAL_STACK_SIZE, &_enc, 1, NULL);
	
	vTaskStartScheduler();
}