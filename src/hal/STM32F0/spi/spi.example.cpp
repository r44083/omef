// Example for STM32F072DISCOVERY development board

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

struct di_poll_task_ctx_t
{
	di &button_1;
	spi &spi;
	gpio &dev1_cs;
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
				uint8_t tx_buff[] = {1, 2, 3, 4, 5};
				int8_t res = ctx->spi.write(tx_buff, sizeof(tx_buff),
					&ctx->dev1_cs);
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
	
	static gpio spi1_mosi_gpio(1, 5, gpio::mode::AF, 1);
	static gpio spi1_miso_gpio(1, 4, gpio::mode::AF, 1);
	static gpio spi1_clk_gpio(1, 3, gpio::mode::AF, 1);
	static gpio dev1_cs(1, 2, gpio::mode::DO, 1);
	
	static dma spi1_tx_dma(dma::DMA_1, dma::CH_3, dma::DIR_MEM_TO_PERIPH,
		dma::INC_SIZE_8);
	static dma spi1_rx_dma(dma::DMA_1, dma::CH_2, dma::DIR_PERIPH_TO_MEM,
		dma::INC_SIZE_8);
	
	static spi spi1(spi::SPI_1, 1000000, spi::CPOL_0, spi::CPHA_0,
		spi::BIT_ORDER_MSB, spi1_tx_dma, spi1_rx_dma, spi1_mosi_gpio,
		spi1_miso_gpio, spi1_clk_gpio);
	
	static di b1_di(b1, 50, 0);
	
	xTaskCreate(heartbeat_task, "heartbeat", configMINIMAL_STACK_SIZE,
		&green_led, 1, NULL);
	
	static di_poll_task_ctx_t di_poll_task_ctx =
		{.button_1 = b1_di, .spi = spi1, .dev1_cs = dev1_cs};
	xTaskCreate(di_poll_task, "di_poll", configMINIMAL_STACK_SIZE,
		&di_poll_task_ctx, 2, NULL);
	
	vTaskStartScheduler();
}