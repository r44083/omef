// Example for STM32F072DISCOVERY board

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
	static gpio b1(0, 0, gpio::MODE_DI, 0);
	static gpio green_led(2, 9, gpio::MODE_DO, 0);
	static gpio uart1_tx_gpio(0, 9, gpio::MODE_AF);
	static gpio uart1_rx_gpio(0, 10, gpio::MODE_AF);
	
	static dma uart1_tx_dma(dma::DMA_1, dma::CH_4, dma::DIR_MEM_TO_PERIPH,
		dma::INC_SIZE_8);
	static dma uart1_rx_dma(dma::DMA_1, dma::CH_5, dma::DIR_PERIPH_TO_MEM,
		dma::INC_SIZE_8);
	
	static uart uart1(uart::UART_1, 115200, uart::STOPBIT_1, uart::PARITY_NONE,
		uart1_tx_dma, uart1_rx_dma, uart1_tx_gpio, uart1_rx_gpio);
	
	static di b1_di(b1, 50, 1);
	b1_di.cb(b1_cb, &uart1);
	
	ASSERT(xTaskCreate(main_task, "main", 50, &green_led, 1, NULL) == pdPASS);
	ASSERT(xTaskCreate(di_task, "di", 200, &b1_di, 2, NULL) == pdPASS);
	
	vTaskStartScheduler();
}

static void b1_cb(di *di, bool state, void *ctx)
{
	if(!state)
		return;
	
	class uart *uart = (class uart *)ctx;
	
	char tx_buff[] = "test";
	char rx_buff[4] = {};
	uint16_t rx_size = sizeof(rx_buff);
	int8_t res = uart->exch(tx_buff, sizeof(tx_buff) - 1, rx_buff, &rx_size);
}
