// Example for STM32F4DISCOVERY development board
// See another example in src/ul/fatfs_diskio/fatfs_diskio.example.cpp

#include "gpio/gpio.hpp"
#include "spi/spi.hpp"
#include "systick/systick.hpp"
#include "drv/sd/sd_spi.hpp"
#include "drv/di/di.hpp"
#include "FreeRTOS.h"
#include "task.h"

using namespace hal;
using namespace drv;

struct di_poll_task_ctx_t
{
	di &card_detection_input;
	sd_spi &sd;
};
static void di_poll_task(void *pvParameters)
{
	di_poll_task_ctx_t *ctx = (di_poll_task_ctx_t *)pvParameters;
	while(1)
	{
		bool new_state;
		if(ctx->card_detection_input.poll_event(new_state))
		{
			if(!new_state)
			{
				int8_t res = ctx->sd.init();
				if(res == sd::RES_OK)
				{
					sd_csd_t csd;
					res = ctx->sd.read_csd(&csd);
				}
			}
		}
		vTaskDelay(1);
	}
}

int main(void)
{
	systick::init();
	static gpio spi1_mosi(0, 7, gpio::mode::AF, 0);
	static gpio spi1_miso(0, 6, gpio::mode::AF, 0);
	static gpio spi1_clk(0, 5, gpio::mode::AF, 0);
	static gpio sd_cs(0, 4, gpio::mode::DO, 1);
	static gpio sd_cd(0, 3, gpio::mode::DI, 1);
	
	static dma spi1_rx_dma(dma::DMA_2, dma::STREAM_0, dma::CH_3,
		dma::DIR_PERIPH_TO_MEM, dma::INC_SIZE_8);
	static dma spi1_tx_dma(dma::DMA_2, dma::STREAM_3, dma::CH_3,
		dma::DIR_MEM_TO_PERIPH, dma::INC_SIZE_8);
	
	static spi spi1(spi::SPI_1, 1000000, spi::CPOL_0, spi::CPHA_0,
		spi::BIT_ORDER_MSB, spi1_tx_dma, spi1_rx_dma, spi1_mosi, spi1_miso,
		spi1_clk);
	
	static sd_spi sd1(spi1, sd_cs, &sd_cd);
	
	static di cd_di(sd_cd, 30, 1);
	
	static di_poll_task_ctx_t di_poll_task_ctx =
		{.card_detection_input = cd_di, .sd = sd1};
	xTaskCreate(di_poll_task, "di_poll", configMINIMAL_STACK_SIZE + 200,
		&di_poll_task_ctx, 1, NULL);
	
	vTaskStartScheduler();
}