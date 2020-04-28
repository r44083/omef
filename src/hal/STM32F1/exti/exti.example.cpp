#include "common/assert.h"
#include "gpio/gpio.hpp"
#include "exti/exti.hpp"
#include "FreeRTOS.h"
#include "task.h"

using namespace hal;

static void main_task(void *pvParameters)
{
	gpio *green_led = (gpio *)pvParameters;
	while(1)
	{
		green_led->toggle();
		vTaskDelay(500);
	}
}

static void exti_cb(exti *exti, void *ctx)
{
	gpio *blue_led = (gpio *)ctx;
	
	blue_led->toggle();
}

int main(void)
{
	static gpio green_led(2, 9, gpio::mode::DO, 0);
	static gpio blue_led(2, 8, gpio::mode::DO, 0);
	static gpio exti1_gpio(0, 0, gpio::mode::DI, 0);
	
	static exti exti1(exti1_gpio, exti::TRIGGER_FALLING);
	exti1.cb(exti_cb, &blue_led);
	exti1.on();
	
	ASSERT(xTaskCreate(main_task, "main", configMINIMAL_STACK_SIZE * 1,
		&green_led, tskIDLE_PRIORITY + 1, NULL) == pdPASS);
	
	vTaskStartScheduler();
}
