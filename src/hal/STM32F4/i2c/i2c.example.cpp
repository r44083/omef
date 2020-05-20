// Example for STM32F4DISCOVERY development board

#include "common/assert.h"
#include "gpio/gpio.hpp"
#include "dma/dma.hpp"
#include "i2c/i2c.hpp"
#include "drv/di/di.hpp"
#include "systick/systick.hpp"
#include "FreeRTOS.h"
#include "task.h"

using namespace hal;
using namespace drv;

struct di_poll_task_ctx_t
{
	di &button_1;
	i2c &i2c;
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
				uint8_t tx_buff[5] = {0xF0, 0xF1, 0xF2, 0xF3, 0xF4};
				uint8_t rx_buff[5];
				
				// lis302
				int8_t res = ctx->i2c.exch(11, tx_buff, sizeof(tx_buff),
					rx_buff, sizeof(rx_buff));
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
	static gpio i2c1_sda(1, 9, gpio::mode::AF, 0);
	static gpio i2c1_scl(1, 6, gpio::mode::AF, 0);
	
	static dma i2c1_tx_dma(dma::DMA_1, dma::STREAM_6, dma::CH_1,
		dma::DIR_MEM_TO_PERIPH, dma::INC_SIZE_8);
	static dma i2c1_rx_dma(dma::DMA_1, dma::STREAM_0, dma::CH_3,
		dma::DIR_PERIPH_TO_MEM, dma::INC_SIZE_8);
	
	static i2c i2c1(i2c::I2C_1, 100000, i2c1_tx_dma, i2c1_rx_dma, i2c1_sda,
		i2c1_scl);
	
	static di b1_di(b1, 50, 1);
	
	xTaskCreate(heartbeat_task, "heartbeat", configMINIMAL_STACK_SIZE,
		&green_led, 1, NULL);
	
	static di_poll_task_ctx_t di_poll_task_ctx =
		{.button_1 = b1_di, .i2c = i2c1};
	xTaskCreate(di_poll_task, "di_poll", configMINIMAL_STACK_SIZE,
		&di_poll_task_ctx, 2, NULL);
	
	vTaskStartScheduler();
}