// Example for STM32F4DISCOVERY development board

#include "gpio/gpio.hpp"
#include "tim/tim.hpp"
#include "exti/exti.hpp"
#include "drv/singlewire/singlewire.hpp"
#include "drv/dht/dht.hpp"
#include "systick/systick.hpp"
#include "FreeRTOS.h"
#include "task.h"

using namespace hal;

static void dht11_task(void *pvParameters)
{
	drv::dht *dht11 = (drv::dht *)pvParameters;
	
	drv::dht::val_t val;
	
	while(1)
	{
		int8_t res = dht11->get(&val);
		
		vTaskDelay(500);
		if(res != drv::dht::RES_OK)
			continue;
		
		/* printf("%02d,%d %%", val.rh_x10 / 10, val.rh_x10 % 10);
		
		char sign = (val.t_x10 >= 0) ? '+' : '-';
		printf("%c%02d,%d C", sign, val.t_x10 / 10, val.t_x10 % 10); */
	}
}

int main(void)
{
	systick::init();
	static gpio dht11_gpio(0, 7, gpio::mode::OD, 1);
	static gpio dht11_exti_gpio(0, 10, gpio::mode::DI, 1);
	
	static tim dht11_tim(tim::TIM_7);
	
	static exti dht11_exti(dht11_exti_gpio);
	
	static drv::singlewire dht11_singlewire(dht11_gpio, dht11_tim, dht11_exti);
	static drv::dht dht11(dht11_singlewire, drv::dht::DHT11);
	
	xTaskCreate(dht11_task, "dht11", configMINIMAL_STACK_SIZE, &dht11, 1, NULL);
	
	vTaskStartScheduler();
}