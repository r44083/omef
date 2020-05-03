#include "common/assert.h"
#include "gpio/gpio.hpp"
#include "wdt/wdt.hpp"
#include "FreeRTOS.h"
#include "task.h"

using namespace hal;

static void main_task(void *pvParameters)
{
	gpio *green_led = (gpio *)pvParameters;
	while(1)
	{
		wdt::reload();
		
		green_led->toggle();
		vTaskDelay(500);
	}
}

int main(void)
{
	static gpio green_led(3, 12, gpio::mode::DO, 0);
	
	wdt::init(1000);
	wdt::on();
	
	ASSERT(xTaskCreate(main_task, "main", configMINIMAL_STACK_SIZE * 1,
		&green_led, tskIDLE_PRIORITY + 1, NULL) == pdPASS);
	
	vTaskStartScheduler();
}