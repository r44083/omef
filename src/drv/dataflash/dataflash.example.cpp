#include "string.h"
#include "gpio/gpio.hpp"
#include "dma/dma.hpp"
#include "spi/spi.hpp"
#include "drv/dataflash/dataflash.hpp"
#include "drv/di/di.hpp"
#include "FreeRTOS.h"
#include "task.h"

using namespace hal;

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
	drv::di *b1_di = (drv::di *)pvParameters;
	
	while(1)
	{
		b1_di->poll();
		vTaskDelay(1);
	}
}

static void b1_cb(drv::di *di, bool state, void *ctx)
{
	if(!state)
		return;
	
	drv::dataflash *at45db = (drv::dataflash *)ctx;
	
	if(at45db->init() != drv::dataflash::RES_OK)
		return;
	
	drv::dataflash::info_t info = at45db->info();
	
	uint8_t buff[info.page_size] = {0x01, 0x02, 0x03, 0x04, 0x05, 0xFF};
	
	int res = at45db->write(buff, 1);
	
	memset(buff, 0, info.page_size);
	res = at45db->read(buff, 1);
	res = at45db->erase(1);
	res = at45db->read(buff, 1);
}

int main(void)
{
	// Example for STM32F4DISCOVERY development board
	static gpio b1(0, 0, gpio::mode::DI, 0);
	static gpio green_led(3, 12, gpio::mode::DO, 0);
	
	static drv::di b1_di(b1, 50, 1);
	
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
	
	static drv::dataflash at45db(spi1, at45db_cs);
	
	b1_di.cb(b1_cb, &at45db);
	
	xTaskCreate(main_task, "main", configMINIMAL_STACK_SIZE * 1, &green_led,
		tskIDLE_PRIORITY + 1, NULL);
	
	xTaskCreate(di_task, "di", configMINIMAL_STACK_SIZE * 8, &b1_di,
		tskIDLE_PRIORITY + 2, NULL);
	
	vTaskStartScheduler();
}
