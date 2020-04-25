// Example for STM32F072DISCOVERY board

#include <string.h>
#include "common/assert.h"
#include "gpio/gpio.hpp"
#include "dma/dma.hpp"
#include "spi/spi.hpp"
#include "exti/exti.hpp"
#include "tim/tim.hpp"
#include "drv/nrf24l01/nrf24l01.hpp"
#include "FreeRTOS.h"
#include "task.h"

using namespace hal;
using namespace drv;

static void nrf_task(void *pvParameters)
{
	static gpio green_led(2, 9, gpio::MODE_DO, 0);
	nrf24l01 *nrf = (nrf24l01 *)pvParameters;
	
	int8_t res = nrf->init();
	if(res)
		return;
	
	res = nrf->open_pipe(1, 0xA5A5);
	if(res)
		return;
	
	while(1)
	{
		nrf24l01::rx_packet_t packet;
		res = nrf->read(packet);
		if(res == nrf24l01::RES_OK && !strncmp((const char *)packet.buff,
			"Hello!", sizeof("Hello!") - 1))
		{
			green_led.toggle();
		}
		
		memset(&packet, 0, sizeof(packet));
	}
	res = nrf->close_pipe(1);
}

int main(void)
{
	static gpio b1(0, 0, gpio::MODE_DI, 0);
	static gpio spi1_mosi_gpio(1, 5, gpio::MODE_AF, 1);
	static gpio spi1_miso_gpio(1, 4, gpio::MODE_AF, 1);
	static gpio spi1_clk_gpio(1, 3, gpio::MODE_AF, 1);
	static gpio nrf24l01_csn(1, 2, gpio::MODE_DO, 1);
	static gpio nrf24l01_ce(1, 6, gpio::MODE_DO, 0);
	static gpio nrf24l01_irq(1, 7, gpio::MODE_DI, 1);
	
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
	
	ASSERT(xTaskCreate(nrf_task, "nrf", 300, &nrf, 2, NULL) == pdPASS);
	
	vTaskStartScheduler();
}
