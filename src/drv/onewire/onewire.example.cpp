// Example for STM32F4DISCOVERY development board

#include "gpio/gpio.hpp"
#include "uart/uart.hpp"
#include "systick/systick.hpp"
#include "drv/di/di.hpp"
#include "drv/onewire/onewire.hpp"
#include "FreeRTOS.h"
#include "task.h"

using namespace hal;
using namespace drv;

static void di_task(void *pvParameters)
{
	di *b1 = (di *)pvParameters;
	while(1)
	{
		b1->poll();
		vTaskDelay(1);
	}
}

static void b1_cb(di *di, bool state, void *ctx)
{
	if(!state)
		return;
	
	onewire *_onewire = (onewire *)ctx;
	
	uint64_t rom = 0;
	int8_t res = _onewire->read_rom(&rom);
	if(res)
		return;
	
	uint8_t tx_buff[3] = {0x01, 0x02, 0x03};
	res = _onewire->tx(rom, tx_buff, sizeof(tx_buff));
}

int main(void)
{
	systick::init();
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
	
	static di b1_di(b1, 50, 1);
	b1_di.cb(b1_cb, &_onewire);
	
	xTaskCreate(di_task, "di", configMINIMAL_STACK_SIZE, &b1_di, 1, NULL);
	
	vTaskStartScheduler();
}