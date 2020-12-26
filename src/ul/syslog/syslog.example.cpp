// Example for STM32F4DISCOVERY development board

#include "common/assert.h"
#include "gpio/gpio.hpp"
#include "dma/dma.hpp"
#include "uart/uart.hpp"
#include "ul/syslog/syslog.hpp"
#include "FreeRTOS.h"
#include "task.h"

using namespace hal;

static void heartbeat_task(void *pvParameters)
{
	gpio *green_led = (gpio *)pvParameters;
	while(1)
	{
		green_led->toggle();
		log().dbg("led toggled to %d", green_led->get());
		vTaskDelay(500);
	}
}

static void uart_write(uint8_t *buff, size_t size, void *ctx)
{
	uart *uart3 = (uart *)ctx;
	
	uart3->write(buff, size);
}

int main(void)
{
	systick::init();
	static gpio green_led(3, 12, gpio::mode::DO, 0);
	static gpio uart3_tx_gpio(3, 8, gpio::mode::AF, 0);
	static gpio uart3_rx_gpio(1, 11, gpio::mode::AF, 0);
	
	static dma uart3_tx_dma(dma::dma_t::DMA_1, dma::stream_t::STREAM_3,
		dma::ch_t::CH_4, dma::dir_t::DIR_MEM_TO_PERIPH, dma::inc_size_t::INC_SIZE_8);
	static dma uart3_rx_dma(dma::dma_t::DMA_1, dma::stream_t::STREAM_1,
		dma::ch_t::CH_4, dma::dir_t::DIR_PERIPH_TO_MEM, dma::inc_size_t::INC_SIZE_8);
	
	static uart uart3(uart::UART_3, 115200, uart::STOPBIT_1, uart::PARITY_NONE,
		uart3_tx_dma, uart3_rx_dma, uart3_tx_gpio, uart3_rx_gpio);
	
	log().add_output(uart_write, &uart3);
	
	xTaskCreate(heartbeat_task, "heartbeat", configMINIMAL_STACK_SIZE + 30,
		&green_led, 1, NULL);
	
	vTaskStartScheduler();
}