// Example for STM32F4DISCOVERY development board

#include "gpio/gpio.hpp"
#include "drv/di/di.hpp"
#include "systick/systick.hpp"
#include "FreeRTOS.h"
#include "task.h"

using namespace hal;
using namespace drv;

struct di_task_ctx_t
{
	di &button_1;
	di &button_2;
	gpio &green_led;
	gpio &blue_led;
};

static void di_task(void *pvParameters)
{
	di_task_ctx_t *ctx = (di_task_ctx_t *)pvParameters;
	while(1)
	{
		bool new_state;
		if(ctx->button_1.poll_event(new_state))
		{
			if(new_state)
				ctx->green_led.toggle();
		}
		if(ctx->button_2.poll_event(new_state))
		{
			if(new_state)
				ctx->blue_led.toggle();
		}
		vTaskDelay(1);
	}
}

int main(void)
{
	systick::init();
	static gpio button_1(0, 0, gpio::mode::DI, 0);
	static gpio button_2(0, 1, gpio::mode::DI, 0);
	static gpio green_led(3, 12, gpio::mode::DO, 0);
	static gpio blue_led(3, 15, gpio::mode::DO, 0);
	
	static di b1_di(button_1, 50, 1);
	static di b2_di(button_2, 50, 1);
	
	di_task_ctx_t di_task_ctx =
	{
		.button_1 = b1_di, .button_2 = b2_di,
		.green_led = green_led, .blue_led = blue_led
	};
	xTaskCreate(di_task, "di", configMINIMAL_STACK_SIZE, &di_task_ctx, 1, NULL);
	
	vTaskStartScheduler();
}