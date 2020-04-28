
#include <string.h>

#include "gpio/gpio.hpp"
#include "drv/sd/sd_spi.hpp"
#include "drv/di/di.hpp"
#include "third_party/FatFs/ff.h"
#include "ul/fatfs_diskio/fatfs_diskio.hpp"
#include "FreeRTOS.h"
#include "task.h"

using namespace hal;
using namespace drv;

static void cd_cb(di *di, bool state, void *ctx);

static void di_task(void *pvParameters)
{
	di *cd_di = (di *)pvParameters;
	while(1)
	{
		cd_di->poll();
		vTaskDelay(1);
	}
}

int main(void)
{
	static gpio b1(0, 0, gpio::mode::DI, 0);
	
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
	
	static drv::sd_spi sd1(spi1, sd_cs, &sd_cd);
	
	static di cd_di(sd_cd, 100, 1);
	cd_di.cb(cd_cb, NULL);
	
	ul::fatfs_diskio_add(0, sd1);
	
	xTaskCreate(di_task, "di", configMINIMAL_STACK_SIZE * 10, &cd_di,
		tskIDLE_PRIORITY + 2, NULL);
	
	vTaskStartScheduler();
}

static void cd_cb(di *di, bool state, void *ctx)
{
	if(state)
	{
		f_unmount("SD");
		return;
	}
	
	FATFS fs;
	FRESULT ff_res = f_mount(&fs, "SD", 1);
	if(ff_res)
	{
		BYTE work_area[FF_MAX_SS] = {};
		ff_res = f_mkfs("SD", FM_FAT32, 0, work_area, sizeof(work_area));
		if(ff_res)
			return;
		ff_res = f_mount(&fs, "SD", 1);
		if(ff_res)
			return;
	}
	
	FIL file;
	ff_res = f_open(&file, "SD:test.txt", FA_WRITE | FA_CREATE_ALWAYS);
	if(ff_res)
		return;
	
	size_t size = strlen("abcdefgh-1234567890");
	ff_res = f_write(&file, "abcdefgh-1234567890", size, &size);
	f_close(&file);
	if(ff_res)
		return;
	
	ff_res = f_open(&file, "SD:test.txt", FA_READ);
	if(ff_res)
		return;
	
	char buff[64] = {};
	size = 0;
	ff_res = f_read(&file, buff, sizeof(buff), &size);
	f_close(&file);
}
