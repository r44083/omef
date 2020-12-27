// Example for STM32F4DISCOVERY development board

#include "common/assert.h"
#include "dma/dma.hpp"
#include "gpio/gpio.hpp"
#include "uart/uart.hpp"
#include "rtc/rtc.hpp"
#include "systick/systick.hpp"
#include "drv/gps/nmea.hpp"
#include "FreeRTOS.h"
#include "task.h"

using namespace hal;

struct gps_task_ctx_t
{
	drv::nmea *nmea;
	hal::gpio *led;
};

static void gps_task(void *pvParameters)
{
	gps_task_ctx_t *ctx = (gps_task_ctx_t *)pvParameters;
	
	gpio *green_led = ctx->led;
	QueueHandle_t from_nmea = ctx->nmea->get_queue();
	drv::nmea::queue_t data;
	
	while(1)
	{
		xQueueReceive(from_nmea, &data, portMAX_DELAY);
		
		if(data.res != drv::nmea::RES_OK || data.type != NMEA_GPRMC)
			continue;
		
		hal::rtc::set(data.gprmc.time);
		
		green_led->toggle();
	}
}

int main(void)
{
	systick::init();
	rtc::init(rtc::CLK_LSI);
	struct tm time = {};
	time.tm_year = 119;
	time.tm_mday = 1;
	rtc::set(time);
	
	static gpio green_led(3, 12, gpio::mode::DO, 0);
	static gpio uart3_tx_gpio(3, 8, gpio::mode::AF, 0);
	static gpio uart3_rx_gpio(1, 11, gpio::mode::AF, 0);
	
	static dma uart3_tx_dma(dma::dma_t::DMA_1, dma::stream_t::STREAM_3,
		dma::ch_t::CH_4, dma::dir_t::DIR_MEM_TO_PERIPH, dma::inc_size_t::INC_SIZE_8);
	static dma uart3_rx_dma(dma::dma_t::DMA_1, dma::stream_t::STREAM_1,
		dma::ch_t::CH_4, dma::dir_t::DIR_PERIPH_TO_MEM, dma::inc_size_t::INC_SIZE_8);
	
	static uart uart3(uart::UART_3, 9600, uart::STOPBIT_1, uart::PARITY_NONE,
		uart3_tx_dma, uart3_rx_dma, uart3_tx_gpio, uart3_rx_gpio);
	
	static drv::nmea gps(uart3, tskIDLE_PRIORITY + 1);
	
	gps_task_ctx_t gps_task_ctx = {.nmea = &gps, .led = &green_led};
	
	xTaskCreate(gps_task, "gps", configMINIMAL_STACK_SIZE + 50, &gps_task_ctx,
		1, NULL);
	
	vTaskStartScheduler();
}