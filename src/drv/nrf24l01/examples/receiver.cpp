// Example for STM32F072DISCOVERY board

#include <string.h>
#include "common/assert.h"
#include "gpio/gpio.hpp"
#include "dma/dma.hpp"
#include "spi/spi.hpp"
#include "exti/exti.hpp"
#include "tim/tim.hpp"
#include "systick/systick.hpp"
#include "drv/nrf24l01/nrf24l01.hpp"
#include "FreeRTOS.h"
#include "task.h"

using namespace hal;
using namespace drv;

static void nrf_task(void *pvParameters)
{
	nrf24l01 *nrf = (nrf24l01 *)pvParameters;
	
	int8_t res = nrf->init();
	ASSERT(res == nrf24l01::RES_OK);
	
	nrf24l01::conf_t conf;
	res = nrf->get_conf(conf);
	ASSERT(res == nrf24l01::RES_OK);
	
	conf.pipe[1].enable = true;
	conf.pipe[1].addr = 0xA5A5;
	conf.pipe[1].auto_ack = true;
	conf.pipe[1].size = nrf24l01::fifo_size;
	conf.datarate = nrf24l01::datarate::_2_Mbps;
	
	res = nrf->set_conf(conf);
	ASSERT(res == nrf24l01::RES_OK);
	
	static gpio green_led(2, 9, gpio::mode::DO, 0);
	while(1)
	{
		nrf24l01::packet_t packet = {};
		res = nrf->read(packet);
		if(res == nrf24l01::RES_OK && !strncmp((const char *)packet.buff,
			"Hello!", sizeof("Hello!") - 1))
		{
			green_led.toggle();
		}
	}
	res = nrf->get_conf(conf);
	ASSERT(res == nrf24l01::RES_OK);
	
	conf.pipe[1].enable = false;
	
	res = nrf->set_conf(conf);
	ASSERT(res == nrf24l01::RES_OK);
}

int main(void)
{
	systick::init();
	static gpio b1(0, 0, gpio::mode::DI, 0);
	static gpio spi1_mosi_gpio(1, 5, gpio::mode::AF, 1);
	static gpio spi1_miso_gpio(1, 4, gpio::mode::AF, 1);
	static gpio spi1_clk_gpio(1, 3, gpio::mode::AF, 1);
	static gpio nrf24l01_csn(1, 8, gpio::mode::DO, 1);
	static gpio nrf24l01_ce(1, 6, gpio::mode::DO, 0);
	static gpio nrf24l01_irq(1, 7, gpio::mode::DI, 1);
	
	static dma spi1_tx_dma(dma::DMA_1, dma::CH_3, dma::DIR_MEM_TO_PERIPH,
		dma::INC_SIZE_8);
	static dma spi1_rx_dma(dma::DMA_1, dma::CH_2, dma::DIR_PERIPH_TO_MEM,
		dma::INC_SIZE_8);
	
	static spi nrf24l01_spi(spi::SPI_1, 4000000, spi::CPOL_0, spi::CPHA_0,
		spi::BIT_ORDER_MSB, spi1_tx_dma, spi1_rx_dma, spi1_mosi_gpio,
		spi1_miso_gpio, spi1_clk_gpio);
	
	static exti nrf24l01_exti(nrf24l01_irq, exti::TRIGGER_FALLING);
	static tim tim6(tim::TIM_6);
	
	static nrf24l01 nrf(nrf24l01_spi, nrf24l01_csn, nrf24l01_ce, nrf24l01_exti,
		tim6);
	
	xTaskCreate(nrf_task, "nrf", configMINIMAL_STACK_SIZE + 20, &nrf, 1, NULL);
	
	vTaskStartScheduler();
}