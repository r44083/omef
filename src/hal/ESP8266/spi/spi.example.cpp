#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gpio/gpio.hpp"
#include "spi/spi.hpp"

struct main_task_ctx_t
{
	hal::spi *spi;
	hal::gpio *cs;
};

static void main_task(void *pvParameters)
{
	main_task_ctx_t *ctx = (main_task_ctx_t *)pvParameters;
	hal::spi &_spi = *ctx->spi;
	hal::gpio &_cs = *ctx->cs;
	
	while(1)
	{
		uint8_t buff[200];
		
		for(size_t i = 0; i < sizeof(buff); i++)
			buff[i] = i;
		
		_spi.write(buff, sizeof(buff), &_cs);
		
		vTaskDelay(1000);
	}
}

extern "C" void app_main(void)
{
	static hal::gpio spi1_miso(0, 12, hal::gpio::mode::AF3);
	static hal::gpio spi1_mosi(0, 13, hal::gpio::mode::AF3);
	static hal::gpio spi1_clk(0, 14, hal::gpio::mode::AF3);
	static hal::gpio spi1_cs(0, 15, hal::gpio::mode::DO);
	
	static hal::spi spi1(hal::spi::SPI_1, 1000000, hal::spi::CPOL_0,
		hal::spi::CPHA_0, hal::spi::BIT_ORDER_MSB, spi1_mosi, spi1_miso,
        spi1_clk);
	
	static main_task_ctx_t ctx = {.spi = &spi1, .cs = &spi1_cs};
	
	xTaskCreate(main_task, "main", 700, &ctx, tskIDLE_PRIORITY + 1, NULL);
}
