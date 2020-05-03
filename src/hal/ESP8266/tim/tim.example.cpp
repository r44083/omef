#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gpio/gpio.hpp"
#include "tim/tim.hpp"

static void main_task(void *pvParameters)
{
	while(1)
	{
		vTaskDelay(500);
	}
}

static void tim_cb(hal::tim *tim, void *ctx)
{
	hal::gpio *blue_led = (hal::gpio *)ctx;
	
	blue_led->toggle();
}

extern "C" void app_main(void)
{
	static hal::gpio blue_led(0, 2, hal::gpio::mode::DO, 1);
	
	static hal::tim tim1(hal::tim::TIM_1);
	tim1.cb(tim_cb, &blue_led);
	tim1.us(200000);
	tim1.start(true);
	
	xTaskCreate(main_task, "main", 300, NULL, tskIDLE_PRIORITY + 1, NULL);
}