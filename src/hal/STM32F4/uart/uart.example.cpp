#include "common/assert.h"
#include "gpio/gpio.hpp"
#include "dma/dma.hpp"
#include "uart/uart.hpp"
#include "drv/di/di.hpp"
#include "FreeRTOS.h"
#include "task.h"

using namespace hal;
using namespace drv;

static void b1_cb(di *di, bool state, void *ctx);

static void main_task(void *pvParameters)
{
	gpio *green_led = (gpio *)pvParameters;
	while(1)
	{
		green_led->toggle();
		vTaskDelay(500);
	}
}

static void di_task(void *pvParameters)
{
	di *b1_di = (di *)pvParameters;
	while(1)
	{
		b1_di->poll();
		vTaskDelay(1);
	}
}

int main(void)
{
	static gpio b1(0, 0, gpio::mode::DI, 0);
	static gpio green_led(3, 12, gpio::mode::DO, 0);
	static gpio uart3_tx_gpio(3, 8, gpio::mode::AF);
	static gpio uart3_rx_gpio(1, 11, gpio::mode::AF);
	
	static dma uart3_tx_dma(dma::DMA_1, dma::STREAM_3, dma::CH_4,
		dma::DIR_MEM_TO_PERIPH, dma::INC_SIZE_8);
	static dma uart3_rx_dma(dma::DMA_1, dma::STREAM_1, dma::CH_4,
		dma::DIR_PERIPH_TO_MEM, dma::INC_SIZE_8);
	
	static uart uart3(uart::UART_3, 115200, uart::STOPBIT_1, uart::PARITY_NONE,
		uart3_tx_dma, uart3_rx_dma, uart3_tx_gpio, uart3_rx_gpio);
	
	static di b1_di(b1, 50, 1);
	b1_di.cb(b1_cb, &uart3);
	
	ASSERT(xTaskCreate(main_task, "main", configMINIMAL_STACK_SIZE * 1,
		&green_led, tskIDLE_PRIORITY + 1, NULL) == pdPASS);
	
	ASSERT(xTaskCreate(di_task, "di", configMINIMAL_STACK_SIZE * 1, &b1_di,
		tskIDLE_PRIORITY + 2, NULL) == pdPASS);
	
	vTaskStartScheduler();
}

static void b1_cb(di *di, bool state, void *ctx)
{
	if(!state)
		return;
	
	uart *uart3 = (uart *)ctx;

	uint8_t tx_buff[] = "test";
	
	int8_t res = uart3->write(tx_buff, sizeof(tx_buff) - 1);
}