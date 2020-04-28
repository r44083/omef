#include "gpio/gpio.hpp"
#include "tim/tim.hpp"
#include "drv/hd44780/hd44780.hpp"
#include "drv/di/di.hpp"
#include "FreeRTOS.h"
#include "task.h"

using namespace hal;
using namespace drv;

static void b1_cb(di *di, bool state, void *ctx);

static void di_task(void *pvParameters)
{
	di *cd_di = (di *)pvParameters;
	while(1)
	{
		cd_di->poll();
		vTaskDelay(1);
	}
}

int main(void)
{
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
	b1_di.cb(b1_cb, &lcd);
	
	xTaskCreate(di_task, "di", configMINIMAL_STACK_SIZE * 3, &b1_di,
		tskIDLE_PRIORITY + 1, NULL);
	
	vTaskStartScheduler();
}

static void b1_cb(di *di, bool state, void *ctx)
{
	if(!state)
		return;
	
	hd44780 *lcd = (hd44780 *)ctx;
	
	lcd->init();
	lcd->print(0, "Test");
	
	// Define custom symbol
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
	lcd->write_cgram(cgram);
	
	lcd->print(64, char(0)); // goto the line 2 and print custom symbol
	lcd->print(20, "Line 3");
	lcd->print(84, "Line 4");
}
