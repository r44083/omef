// Example for STM32F072DISCOVERY development board

#include "common/assert.h"
#include "gpio/gpio.hpp"
#include "wdt/wdt.hpp"
#include "systick/systick.hpp"
#include "FreeRTOS.h"
#include "task.h"

using namespace hal;

static void heartbeat_task(void *pvParameters)
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
	systick::init();
	static gpio green_led(2, 9, gpio::mode::DO, 0);
	
	wdt::init(1000);
	wdt::on();
	
	xTaskCreate(heartbeat_task, "heartbeat", configMINIMAL_STACK_SIZE,
		&green_led, 1, NULL);
	
	vTaskStartScheduler();
}