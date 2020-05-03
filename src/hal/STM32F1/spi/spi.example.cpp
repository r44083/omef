#include "common/assert.h"
#include "gpio/gpio.hpp"
#include "dma/dma.hpp"
#include "spi/spi.hpp"
#include "drv/di/di.hpp"
#include "FreeRTOS.h"
#include "task.h"

using namespace hal;
using namespace drv;

typedef struct
{
	spi *spi1;
	gpio *cs;
} b1_ctx_t;

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
	static gpio green_led(2, 9, gpio::mode::DO, 0);
	
	static gpio spi1_mosi_gpio(0, 7, gpio::mode::AF);
	static gpio spi1_miso_gpio(0, 6, gpio::mode::AF);
	static gpio spi1_clk_gpio(0, 5, gpio::mode::AF);
	static gpio dev1_cs(0, 4, gpio::mode::DO);
	
	static dma spi1_tx_dma(dma::DMA_1, dma::CH_3, dma::DIR_MEM_TO_PERIPH,
		dma::INC_SIZE_8);
	static dma spi1_rx_dma(dma::DMA_1, dma::CH_2, dma::DIR_PERIPH_TO_MEM,
		dma::INC_SIZE_8);
	
	static spi spi1(spi::SPI_1, 1000000, spi::CPOL_0, spi::CPHA_0,
		spi::BIT_ORDER_MSB, spi1_tx_dma, spi1_rx_dma, spi1_mosi_gpio,
		spi1_miso_gpio, spi1_clk_gpio);
	
	static di b1_di(b1, 50, 1);
	
	static b1_ctx_t b1_ctx = {.spi1 = &spi1, .cs = &dev1_cs};
	b1_di.cb(b1_cb, &b1_ctx);
	
	ASSERT(xTaskCreate(main_task, "main", 100, &green_led, 1, NULL) == pdPASS);
	
	ASSERT(xTaskCreate(di_task, "di", 500, &b1_di, 2, NULL) == pdPASS);
	
	vTaskStartScheduler();
}

static void b1_cb(di *di, bool state, void *ctx)
{
	if(!state)
		return;
	
	b1_ctx_t *b1_ctx = (b1_ctx_t *)ctx;
	
	uint8_t tx_buff[] = "test";
	
	int8_t res = b1_ctx->spi1->write(tx_buff, sizeof(tx_buff) - 1, b1_ctx->cs);
}