// Example for STM32F4DISCOVERY development board

#include "gpio/gpio.hpp"
#include "tim/tim.hpp"
#include "systick/systick.hpp"
#include "drv/hd44780/hd44780.hpp"
#include "drv/di/di.hpp"
#include "FreeRTOS.h"
#include "task.h"

using namespace hal;
using namespace drv;

struct di_poll_task_ctx_t
{
	di &button_1;
	hd44780 &lcd;
};

static void di_poll_task(void *pvParameters)
{
	di_poll_task_ctx_t *ctx = (di_poll_task_ctx_t *)pvParameters;
	
	ctx->lcd.init();
	ctx->lcd.print(0, "Test");
	
	uint8_t cgram[8][8] =
	{
		{
			0b00011000,
			0b00001110,
			0b00000110,
			0b00000111,
			0b00000111,
			0b00000110,
			0b00001100,
			0b00011000
		}
	};
	ctx->lcd.write_cgram(cgram);
	
	ctx->lcd.print(64, char(0)); // goto the line 2 and print custom symbol
	ctx->lcd.print(20, "Line 3");
	ctx->lcd.print(84, "Line 4");
	
	while(1)
	{
		bool new_state;
		if(ctx->button_1.poll_event(new_state))
		{
			if(new_state)
			{
				ctx->lcd.print(0, "Test");
			}
		}
		vTaskDelay(1);
	}
}

int main(void)
{
	systick::init();
	static gpio b1(0, 0, gpio::mode::DI);
	
	static gpio rs(0, 5, gpio::mode::DO);
	static gpio rw(0, 4, gpio::mode::DO);
	static gpio e(0, 3, gpio::mode::DO);
	static gpio db4(0, 6, gpio::mode::DO);
	static gpio db5(0, 7, gpio::mode::DO);
	static gpio db6(0, 8, gpio::mode::DO);
	static gpio db7(0, 10, gpio::mode::DO);
	
	static tim tim6(tim::TIM_6);
	
	static hd44780 lcd(rs, rw, e, db4, db5, db6, db7, tim6);
	
	static di b1_di(b1, 50, 1);
	
	di_poll_task_ctx_t di_poll_task_ctx = {.button_1 = b1_di, .lcd = lcd};
	xTaskCreate(di_poll_task, "di_poll", configMINIMAL_STACK_SIZE,
		&di_poll_task_ctx, 1, NULL);
	
	vTaskStartScheduler();
}