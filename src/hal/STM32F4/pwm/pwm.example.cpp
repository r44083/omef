// Example for STM32F4DISCOVERY development board

#include "common/assert.h"
#include "gpio/gpio.hpp"
#include "pwm/pwm.hpp"
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

int main(void)
{
	systick::init();
	static gpio green_led(3, 12, gpio::mode::DO, 0);
	static gpio pwm2_ch2_gpio(0, 1, gpio::mode::AF, 0);
	
	static pwm pwm2_ch2(tim::TIM_2, pwm::CH_2, pwm2_ch2_gpio);
	pwm2_ch2.freq(100000);
	pwm2_ch2.duty(20);
	pwm2_ch2.start();
	
	xTaskCreate(heartbeat_task, "heartbeat", configMINIMAL_STACK_SIZE,
		&green_led, 1, NULL);
	
	vTaskStartScheduler();
}