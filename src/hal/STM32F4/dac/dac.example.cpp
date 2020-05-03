// Example for STM32F4DISCOVERY development board

#include "common/assert.h"
#include "dac/dac.hpp"
#include "gpio/gpio.hpp"
#include "tim/tim.hpp"
#include "systick/systick.hpp"
#include "FreeRTOS.h"
#include "task.h"

using namespace hal;

static void heartbeat_task(void *pvParameters)
{
	gpio *green_led = (gpio *)pvParameters;
	while(1)
	{
		green_led->toggle();
		vTaskDelay(500);
	}
}

static void tim_cb(tim *tim, void *ctx)
{
	dac *dac1 = (dac *)ctx;
	
	static const uint16_t sin[] =
	{
		2047, 2447, 2831, 3185, 3498, 3750, 3939, 4056,
		4095, 4056, 3939, 3750, 3495, 3185, 2831, 2447,
		2047, 1647, 1263,  909,  599,  344,  155,   38,
		   0,   38,  155,  344,  599,  909, 1263, 1647
	};
	static uint8_t cnt = 0;
	
	dac1->set(sin[cnt++]);
	if(cnt == (sizeof(sin) / sizeof(sin[0])))
		cnt = 0;
}

int main(void)
{
	systick::init();
	static gpio dac1_gpio(0, 4, gpio::mode::AN, 0);
	static gpio green_led(3, 12, gpio::mode::DO, 0);
	
	static dac dac1(dac::DAC_1, dac::ALIGN_R_12, dac1_gpio);
	
	static tim tim6(tim::TIM_6);
	tim6.cb(tim_cb, &dac1);
	tim6.us(32); // Sine wave frequency is 1 kHz (sin[32])
	tim6.start(true);
	
	xTaskCreate(heartbeat_task, "heartbeat", configMINIMAL_STACK_SIZE,
		&green_led, 1, NULL);
	
	vTaskStartScheduler();
}