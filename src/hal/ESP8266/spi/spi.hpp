#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "gpio/gpio.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

namespace hal { class spi; }
// For internal use only! (called from ISR)
extern "C" void spi_irq_hndlr(hal::spi *obj);

namespace hal
{
class spi
{
	public:
		enum spi_t
		{
			SPI_0, // External flash memory
			SPI_1  // HSPI
		};

		enum cpol_t
		{
			CPOL_0,
			CPOL_1
		};

		enum cpha_t
		{
			CPHA_0,
			CPHA_1
		};

		enum bit_order_t
		{
			BIT_ORDER_MSB,
			BIT_ORDER_LSB
		};

		enum res_t
		{
			RES_OK   =  0,
			RES_FAIL = -1
		};
		
		spi(spi_t spi, uint32_t baud, cpol_t cpol, cpha_t cpha,
			bit_order_t bit_order, gpio &mosi, gpio &miso, gpio &clk);
		~spi();
		
		void baud(uint32_t baud);
		uint32_t baud() const { return _baud; }
		void cpol(cpol_t cpol);
		cpol_t cpol() const { return _cpol; }
		void cpha(cpha_t cpha);
		cpha_t cpha() const { return _cpha; }
		void bit_order(bit_order_t bit_order);
		bit_order_t bit_order() const { return _bit_order; };
		
		int8_t write(void *buff, size_t size, gpio *cs = NULL);
		int8_t write(uint8_t byte, gpio *cs = NULL);
		int8_t read(void *buff, size_t size, gpio *cs = NULL);
		int8_t exch(void *buff_tx, void *buff_rx, size_t size, gpio *cs = NULL);
		
		spi &operator = (const spi &);
	
	private:
		spi_t _spi;
		uint32_t _baud;
		cpol_t _cpol;
		cpha_t _cpha;
		bit_order_t _bit_order;
		SemaphoreHandle_t api_lock;
		TaskHandle_t task;
		res_t irq_res;
		gpio &_mosi;
		gpio &_miso;
		gpio &_clk;
		gpio *_cs;
		uint8_t *tx_buff;
		uint8_t *rx_buff;
		size_t remain;
		
		enum spi_div_t
		{
			SPI_DIV_80MHz = 1,
			SPI_DIV_40MHz = 2,
			SPI_DIV_20MHz = 4,
			SPI_DIV_16MHz = 5,
			SPI_DIV_10MHz = 8,
			SPI_DIV_8MHz = 10,
			SPI_DIV_5MHz = 16,
			SPI_DIV_4MHz = 20,
			SPI_DIV_2MHz = 40
		};
		static void calc_clk(spi_t spi, uint32_t baud, spi_div_t &div,
			uint16_t &prediv);
		void handle_spi0_enabled_irq(spi_t spi);
		friend void ::spi_irq_hndlr(spi *obj);
};
}