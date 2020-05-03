// Example for STM32F4DISCOVERY development board

#include "gpio/gpio.hpp"
#include "drv/di/di.hpp"
#include "systick/systick.hpp"
#include "FreeRTOS.h"
#include "task.h"

using namespace hal;
using namespace drv;

static void di_task(void *pvParameters)
{
	di *b1_di = (di *)pvParameters;
	while(1)
	{
		b1_di->poll();
		vTaskDelay(1);
	}
}

static void b1_cb(di *di, bool state, void *ctx)
{
	if(!state)
		return;
	
	gpio *green_led = (gpio *)ctx;
	green_led->toggle();
}

int main(void)
{
	systick::init();
	static gpio b1(0, 0, gpio::mode::DI, 0);
	static gpio green_led(3, 12, gpio::mode::DO, 0);
	
	static di b1_di(b1, 50, 1);
	b1_di.cb(b1_cb, &green_led);
	
	xTaskCreate(di_task, "di", configMINIMAL_STACK_SIZE, &b1_di, 1, NULL);
	
	vTaskStartScheduler();
}