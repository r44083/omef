// Exapmle for STM32VLDISCOVERY board

#include <string.h>
#include "common/assert.h"
#include "gpio/gpio.hpp"
#include "dma/dma.hpp"
#include "spi/spi.hpp"
#include "exti/exti.hpp"
#include "tim/tim.hpp"
#include "systick/systick.hpp"
#include "drv/di/di.hpp"
#include "drv/nrf24l01/nrf24l01.hpp"
#include "FreeRTOS.h"
#include "task.h"

using namespace hal;
using namespace drv;

struct di_poll_task_ctx_t
{
	di &button_1;
	nrf24l01 &nrf;
	gpio &led;
};
static void di_poll_task(void *pvParameters)
{
	di_poll_task_ctx_t *ctx = (di_poll_task_ctx_t *)pvParameters;
	
	int8_t res = ctx->nrf.init();
	ASSERT(res == nrf24l01::RES_OK);
	
	nrf24l01::conf_t conf;
	res = ctx->nrf.get_conf(conf);
	ASSERT(res == nrf24l01::RES_OK);
	
	conf.tx_addr = 0xA5A5;
	conf.tx_auto_ack = true;
	conf.dyn_payload = true;
	conf.datarate = nrf24l01::datarate::_2_Mbps;
	
	res = ctx->nrf.set_conf(conf);
	ASSERT(res == nrf24l01::RES_OK);
	
	while(1)
	{
		bool new_state;
		if(ctx->button_1.poll_event(new_state))
		{
			if(new_state)
			{
				char tx_buff[sizeof("Hello!")] = {};
				strncpy(tx_buff, "Hello!", sizeof(tx_buff));
				nrf24l01::packet_t ack = {};
				
				res = ctx->nrf.write(tx_buff, sizeof(tx_buff), &ack);
				ctx->nrf.power_down();
				
				if(res == nrf24l01::RES_OK)
					ctx->led.toggle();
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
	static gpio spi1_mosi_gpio(0, 7, gpio::mode::AF, 1);
	static gpio spi1_miso_gpio(0, 6, gpio::mode::AF, 1);
	static gpio spi1_clk_gpio(0, 5, gpio::mode::AF, 1);
	static gpio nrf24l01_csn(0, 4, gpio::mode::DO, 1);
	static gpio nrf24l01_ce(0, 3, gpio::mode::DO, 0);
	static gpio nrf24l01_irq(0, 2, gpio::mode::DI, 1);
	
	static dma spi1_tx_dma(dma::DMA_1, dma::CH_3, dma::DIR_MEM_TO_PERIPH,
		dma::INC_SIZE_8);
	static dma spi1_rx_dma(dma::DMA_1, dma::CH_2, dma::DIR_PERIPH_TO_MEM,
		dma::INC_SIZE_8);
	
	static spi nrf24l01_spi(spi::SPI_1, 1000000, spi::CPOL_0, spi::CPHA_0,
		spi::BIT_ORDER_MSB, spi1_tx_dma, spi1_rx_dma, spi1_mosi_gpio,
		spi1_miso_gpio, spi1_clk_gpio);
	
	static exti nrf24l01_exti(nrf24l01_irq, exti::TRIGGER_FALLING);
	static tim tim6(tim::TIM_6);
	
	static nrf24l01 nrf(nrf24l01_spi, nrf24l01_csn, nrf24l01_ce, nrf24l01_exti,
		tim6);
	
	static di b1_di(b1, 50, 0);
	
	static di_poll_task_ctx_t di_poll_task_ctx =
		{.button_1 = b1_di, .nrf = nrf, .led = green_led};
	xTaskCreate(di_poll_task, "di_poll", configMINIMAL_STACK_SIZE + 20,
		&di_poll_task_ctx, 1, NULL);
	
	vTaskStartScheduler();
}