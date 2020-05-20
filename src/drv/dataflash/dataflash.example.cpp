// Example for STM32F4DISCOVERY development board

#include "string.h"
#include "common/assert.h"
#include "gpio/gpio.hpp"
#include "dma/dma.hpp"
#include "spi/spi.hpp"
#include "systick/systick.hpp"
#include "drv/dataflash/dataflash.hpp"
#include "drv/di/di.hpp"
#include "FreeRTOS.h"
#include "task.h"

using namespace hal;
using namespace drv;

struct di_poll_task_ctx_t
{
	di &button_1;
	dataflash &at45db;
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
	
	int8_t res = ctx->at45db.init();
	drv::dataflash::info_t info = ctx->at45db.info();
	
	while(1)
	{
		bool new_state;
		if(ctx->button_1.poll_event(new_state))
		{
			if(new_state)
			{
				uint8_t buff[info.page_size] = {0x01, 0x02, 0x03, 0x04, 0x05};
				res = ctx->at45db.write(buff, 1);
				memset(buff, 0, info.page_size);
				res = ctx->at45db.read(buff, 1);
				res = ctx->at45db.erase(1);
				res = ctx->at45db.read(buff, 1);
			}
		}
		vTaskDelay(1);
	}
}

int main(void)
{
	systick::init();
	static gpio b1(0, 0, gpio::mode::DI, 0);
	static gpio green_led(3, 12, gpio::mode::DO, 0);
	
	static gpio spi1_mosi(0, 7, gpio::mode::AF, 0);
	static gpio spi1_miso(0, 6, gpio::mode::AF, 0);
	static gpio spi1_clk(0, 5, gpio::mode::AF, 0);
	static gpio at45db_cs(0, 4, gpio::mode::DO, 1);
	
	static dma spi1_rx_dma(dma::DMA_2, dma::STREAM_0, dma::CH_3,
		dma::DIR_PERIPH_TO_MEM, dma::INC_SIZE_8);
	static dma spi1_tx_dma(dma::DMA_2, dma::STREAM_3, dma::CH_3,
		dma::DIR_MEM_TO_PERIPH, dma::INC_SIZE_8);
	
	static spi spi1(spi::SPI_1, 1000000, spi::CPOL_0, spi::CPHA_0,
		spi::BIT_ORDER_MSB, spi1_tx_dma, spi1_rx_dma, spi1_mosi, spi1_miso,
		spi1_clk);
	
	static di b1_di(b1, 50, 1);
	static dataflash at45db(spi1, at45db_cs);
	
	xTaskCreate(heartbeat_task, "heartbeat", configMINIMAL_STACK_SIZE,
		&green_led, 1, NULL);
	
	di_poll_task_ctx_t di_poll_task_ctx = {.button_1 = b1_di, .at45db = at45db};
	xTaskCreate(di_poll_task, "di_poll", configMINIMAL_STACK_SIZE + 100,
		&di_poll_task_ctx, 2, NULL);
	
	vTaskStartScheduler();
}