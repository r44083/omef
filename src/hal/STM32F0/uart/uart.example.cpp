// Example for STM32F072DISCOVERY development board

#include "common/assert.h"
#include "gpio/gpio.hpp"
#include "dma/dma.hpp"
#include "uart/uart.hpp"
#include "drv/di/di.hpp"
#include "systick/systick.hpp"
#include "FreeRTOS.h"
#include "task.h"

using namespace hal;
using namespace drv;

struct di_poll_task_ctx_t
{
	di &button_1;
	uart &uart;
};

static void heartbeat_task(void *pvParameters)
{
	gpio *green_led = (gpio *)pvParameters;
	while(1)
	{
		green_led->toggle();
		vTaskDelay(500);
	}
}

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
				char tx_buff[] = "test";
				char rx_buff[4] = {};
				uint16_t rx_size = sizeof(rx_buff);
				int8_t res = ctx->uart.exch(tx_buff, sizeof(tx_buff) - 1,
					rx_buff, &rx_size);
			}
		}
		vTaskDelay(1);
	}
}

int main(void)
{
	systick::init();
	static gpio b1(0, 0, gpio::mode::DI, 0);
	static gpio green_led(2, 9, gpio::mode::DO, 0);
	static gpio uart1_tx_gpio(0, 9, gpio::mode::AF);
	static gpio uart1_rx_gpio(0, 10, gpio::mode::AF);
	
	static dma uart1_tx_dma(dma::DMA_1, dma::CH_4, dma::DIR_MEM_TO_PERIPH,
		dma::INC_SIZE_8);
	static dma uart1_rx_dma(dma::DMA_1, dma::CH_5, dma::DIR_PERIPH_TO_MEM,
		dma::INC_SIZE_8);
	
	static uart uart1(uart::UART_1, 115200, uart::STOPBIT_1, uart::PARITY_NONE,
		uart1_tx_dma, uart1_rx_dma, uart1_tx_gpio, uart1_rx_gpio);
	
	static di b1_di(b1, 50, 1);
	
	xTaskCreate(heartbeat_task, "heartbeat", configMINIMAL_STACK_SIZE,
		&green_led, 1, NULL);
	
	static di_poll_task_ctx_t di_poll_task_ctx =
		{.button_1 = b1_di, .uart = uart1};
	xTaskCreate(di_poll_task, "di_poll", configMINIMAL_STACK_SIZE,
		&di_poll_task_ctx, 2, NULL);
	
	vTaskStartScheduler();
}