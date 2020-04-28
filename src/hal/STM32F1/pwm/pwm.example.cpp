#include "common/assert.h"
#include "gpio/gpio.hpp"
#include "pwm/pwm.hpp"
#include "tim/tim.hpp"
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

int main(void)
{
	static gpio green_led(2, 9, gpio::mode::DO, 0);
	static gpio pwm3_ch3_gpio(2, 8, gpio::mode::AF, 0); // blue led
	
	static pwm pwm3_ch3(tim::TIM_3, pwm::CH_3, pwm3_ch3_gpio);
	pwm3_ch3.freq(100000);
	pwm3_ch3.duty(20);
	pwm3_ch3.start();
	
	ASSERT(xTaskCreate(main_task, "main", configMINIMAL_STACK_SIZE * 1, &green_led,
		tskIDLE_PRIORITY + 1, NULL) == pdPASS);
	
	vTaskStartScheduler();
}
