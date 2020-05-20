// Example for STM32F4DISCOVERY development board

#include "gpio/gpio.hpp"
#include "tim/tim.hpp"
#include "exti/exti.hpp"
#include "systick/systick.hpp"
#include "drv/di/di.hpp"
#include "drv/singlewire/singlewire.hpp"
#include "FreeRTOS.h"
#include "task.h"

using namespace hal;
using namespace drv;

struct di_poll_task_ctx_t
{
	di &button_1;
	singlewire dht;
};
static void di_poll_task(void *pvParameters)
{
	di_poll_task_ctx_t *ctx = (di_poll_task_ctx_t *)pvParameters;
	while(1)
	{
		bool new_state;
		if(ctx->button_1.poll_event(new_state))
		{
			if(new_state)
			{
				uint8_t buff[5];
				int8_t res = ctx->dht.read(buff, sizeof(buff));
			}
		}
		vTaskDelay(1);
	}
}

int main(void)
{
	systick::init();
	static gpio b1(0, 0, gpio::mode::DI, 0);
	static gpio singlewire_gpio(0, 7, gpio::mode::OD, 1);
	static gpio singlewire_exti_gpio(0, 10, gpio::mode::DI, 1);
	
	static tim singlewire_tim(tim::TIM_7);
	
	static exti singlewire_exti(singlewire_exti_gpio);
	
	static singlewire _singlewire(singlewire_gpio, singlewire_tim,
		singlewire_exti);
	
	static di b1_di(b1, 50, 1);
	
	static di_poll_task_ctx_t di_poll_task_ctx =
		{.button_1 = b1_di, .dht = _singlewire};
	xTaskCreate(di_poll_task, "di_poll", configMINIMAL_STACK_SIZE,
		&di_poll_task_ctx, 1, NULL);
	
	vTaskStartScheduler();
}