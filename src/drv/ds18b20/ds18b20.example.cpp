
#include "gpio/gpio.hpp"
#include "uart/uart.hpp"
#include "drv/di/di.hpp"
#include "drv/onewire/onewire.hpp"
#include "drv/ds18b20/ds18b20.hpp"
#include "FreeRTOS.h"
#include "task.h"

using namespace hal;
using namespace drv;

static void b1_cb(di *di, bool state, void *ctx);

static void di_task(void *pvParameters)
{
	di *b1 = (di *)pvParameters;
	while(1)
	{
		b1->poll();
		vTaskDelay(1);
	}
}

int main(void)
{
	static gpio b1(0, 0, gpio::mode::DI, 0);
	static gpio uart3_tx_gpio(3, 8, gpio::mode::AF, 0);
	static gpio uart3_rx_gpio(1, 11, gpio::mode::AF, 0);
	
	static dma uart3_tx_dma(dma::DMA_1, dma::STREAM_3, dma::CH_4,
		dma::DIR_MEM_TO_PERIPH, dma::INC_SIZE_8);
	static dma uart3_rx_dma(dma::DMA_1, dma::STREAM_1, dma::CH_4,
		dma::DIR_PERIPH_TO_MEM, dma::INC_SIZE_8);
	
	static uart uart3(uart::UART_3, 115200, uart::STOPBIT_1, uart::PARITY_NONE,
		uart3_tx_dma, uart3_rx_dma, uart3_tx_gpio, uart3_rx_gpio);
	
	static onewire _onewire(uart3);
	static ds18b20 _ds18b20(_onewire);
	
	static di b1_di(b1, 50, 1);
	b1_di.cb(b1_cb, &_ds18b20);
	
	xTaskCreate(di_task, "di", configMINIMAL_STACK_SIZE * 3, &b1_di,
		tskIDLE_PRIORITY + 1, NULL);
	
	vTaskStartScheduler();
}

static void b1_cb(di *di, bool state, void *ctx)
{
	if(!state)
		return;
	
	ds18b20 *_ds18b20 = (ds18b20 *)ctx;
	
	float temp = 0;
	int8_t res = _ds18b20->get_temp(0, &temp);
}
