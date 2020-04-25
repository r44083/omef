// Exapmle for STM32VLDISCOVERY board

#include <string.h>
#include "common/assert.h"
#include "gpio/gpio.hpp"
#include "dma/dma.hpp"
#include "spi/spi.hpp"
#include "exti/exti.hpp"
#include "tim/tim.hpp"
#include "drv/di/di.hpp"
#include "drv/nrf24l01/nrf24l01.hpp"
#include "FreeRTOS.h"
#include "task.h"

using namespace hal;
using namespace drv;

static void b1_cb(di *di, bool state, void *ctx);

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
	static gpio b1(0, 0, gpio::MODE_DI, 0);
	static gpio spi1_mosi_gpio(0, 7, gpio::MODE_AF, 1);
	static gpio spi1_miso_gpio(0, 6, gpio::MODE_AF, 1);
	static gpio spi1_clk_gpio(0, 5, gpio::MODE_AF, 1);
	static gpio nrf24l01_csn(0, 4, gpio::MODE_DO, 1);
	static gpio nrf24l01_ce(0, 3, gpio::MODE_DO, 0);
	static gpio nrf24l01_irq(0, 2, gpio::MODE_DI, 1);
	
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
	
	static di b1_di(b1, 50, 0);
	b1_di.cb(b1_cb, &nrf);
	
	ASSERT(xTaskCreate(di_task, "di", 400, &b1_di, 2, NULL) == pdPASS);
	
	vTaskStartScheduler();
}

static void b1_cb(di *di, bool state, void *ctx)
{
	if(!state)
		return;
	
	static gpio green_led(2, 9, gpio::MODE_DO, 0);
	
	nrf24l01 *nrf = (nrf24l01 *)ctx;
	int8_t res = nrf->init();
	res = nrf->tx_addr(0xA5A5);
	
	char tx_buff[32] = {};
	strncpy(tx_buff, "Hello!", sizeof(tx_buff));
	res = nrf->write(tx_buff, sizeof(tx_buff));
	nrf->power_down();
	
	if(res == nrf24l01::RES_OK)
		green_led.toggle();
}
