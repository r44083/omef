// Example for STM32F4DISCOVERY development board

#include "common/assert.h"
#include "gpio/gpio.hpp"
#include "dma/dma.hpp"
#include "spi/spi.hpp"
#include "drv/di/di.hpp"
#include "systick/systick.hpp"
#include "FreeRTOS.h"
#include "task.h"

using namespace hal;
using namespace drv;

static void heartbeat_task(void *pvParameters)
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

struct b1_ctx_t
{
	spi *lis302_spi;
	gpio *lis302_cs;
};

static void b1_cb(di *di, bool state, void *ctx)
{
	if(!state)
		return;
	
	b1_ctx_t *b1_ctx = (b1_ctx_t *)ctx;
	
	uint8_t tx_buff[5] = {0xF0, 0xF1, 0xF2, 0xF3, 0xF4};
	uint8_t rx_buff[5];
	int8_t res = b1_ctx->lis302_spi->exch(tx_buff, rx_buff, sizeof(tx_buff),
		b1_ctx->lis302_cs);
}

int main(void)
{
	systick::init();
	static gpio b1(0, 0, gpio::mode::DI, 0);
	static gpio green_led(3, 12, gpio::mode::DO, 0);
	static gpio spi1_mosi(0, 7, gpio::mode::AF);
	static gpio spi1_miso(0, 6, gpio::mode::AF);
	static gpio spi1_clk(0, 5, gpio::mode::AF);
	static gpio lis302_cs(4, 3, gpio::mode::DO, 1);
	
	static dma spi1_tx_dma(dma::DMA_2, dma::STREAM_3, dma::CH_3,
		dma::DIR_MEM_TO_PERIPH, dma::INC_SIZE_8);
	static dma spi1_rx_dma(dma::DMA_2, dma::STREAM_0, dma::CH_3,
		dma::DIR_PERIPH_TO_MEM, dma::INC_SIZE_8);
	
	static spi spi1(spi::SPI_1, 10000000, spi::CPOL_0, spi::CPHA_0,
		spi::BIT_ORDER_MSB, spi1_tx_dma, spi1_rx_dma, spi1_mosi, spi1_miso,
		spi1_clk);
	
	static di b1_di(b1, 50, 1);
	
	b1_ctx_t b1_ctx = {.lis302_spi = &spi1, .lis302_cs = &lis302_cs};
	b1_di.cb(b1_cb, &b1_ctx);
	
	xTaskCreate(heartbeat_task, "heartbeat", configMINIMAL_STACK_SIZE,
		&green_led, 1, NULL);
	xTaskCreate(di_task, "di", configMINIMAL_STACK_SIZE, &b1_di, 2, NULL);
	
	vTaskStartScheduler();
}