// Example for nRF52-DK development board

#include "FreeRTOS.h"
#include "task.h"
#include "gpio/gpio.hpp"

using namespace hal;

static void heartbeat_task(void *pvParameters)
{
	gpio &led_1 = *(gpio *)pvParameters;
	
	while(1)
	{
		led_1.toggle();
		vTaskDelay(500);
	}
}

int main(void)
{
	static gpio led_1(0, 17, gpio::mode::DO, true);
	
	xTaskCreate(heartbeat_task, "heartbeat", configMINIMAL_STACK_SIZE, &led_1,
		1, NULL);
	
	vTaskStartScheduler();
}