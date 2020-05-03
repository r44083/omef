// Example for STM32F4DISCOVERY development board

#include "common/assert.h"
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
	gpio *blue_led = (gpio *)ctx;
	
	blue_led->toggle();
}

int main(void)
{
	systick::init();
	static gpio green_led(3, 12, gpio::mode::DO, 0);
	static gpio blue_led(3, 15, gpio::mode::DO, 0);
	
	static tim tim6(tim::TIM_6);
	tim6.cb(tim_cb, &blue_led);
	tim6.us(100);
	tim6.start(true);
	
	xTaskCreate(heartbeat_task, "heartbeat", configMINIMAL_STACK_SIZE,
		&green_led, 1, NULL);
	
	vTaskStartScheduler();
}