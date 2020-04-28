#include "gpio/gpio.hpp"
#include "tim/tim.hpp"
#include "exti/exti.hpp"
#include "drv/di/di.hpp"
#include "drv/singlewire/singlewire.hpp"
#include "FreeRTOS.h"
#include "task.h"

using namespace hal;

static void b1_cb(drv::di *di, bool state, void *ctx);

static void di_task(void *pvParameters)
{
	drv::di *b1_di = (drv::di *)pvParameters;
	while(1)
	{
		b1_di->poll();
		vTaskDelay(1);
	}
}

int main(void)
{
	// Example for STM32F4DISCOVERY development board
	static gpio b1(0, 0, gpio::mode::DI, 0);
	static gpio singlewire_gpio(0, 7, gpio::mode::OD, 1);
	static gpio singlewire_exti_gpio(0, 10, gpio::mode::DI, 1);
	
	static tim singlewire_tim(tim::TIM_7);
	
	static exti singlewire_exti(singlewire_exti_gpio);
	
	static drv::singlewire _singlewire(singlewire_gpio, singlewire_tim,
		singlewire_exti);
	
	static drv::di b1_di(b1, 50, 1);
	b1_di.cb(b1_cb, &_singlewire);
	
	xTaskCreate(di_task, "di", configMINIMAL_STACK_SIZE * 1, &b1_di,
		tskIDLE_PRIORITY + 1, NULL);
	
	vTaskStartScheduler();
}

static void b1_cb(drv::di *di, bool state, void *ctx)
{
	if(!state)
		return;
	
	drv::singlewire *_singlewire = (drv::singlewire *)ctx;
	
	uint8_t buff[5];
	int8_t res = _singlewire->read(buff, sizeof(buff));
}
