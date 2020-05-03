#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "gpio/gpio.hpp"
#include "dma/dma.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

namespace hal { class i2c; }
// For internal use only! (called from ISR)
extern "C" void tx_hndlr(hal::i2c *obj, BaseType_t *hi_task_woken);
extern "C" void rx_hndlr(hal::i2c *obj, BaseType_t *hi_task_woken);
extern "C" void err_hndlr(hal::i2c *obj, int8_t err, BaseType_t *hi_task_woken);
extern "C" void i2c_event_irq_hndlr(hal::i2c *obj);
extern "C" void i2c_error_irq_hndlr(hal::i2c *obj);

namespace hal
{
class i2c
{
	public:
		enum i2c_t
		{
			I2C_1,
			I2C_2,
			I2C_3,
			I2C_END
		};

		enum res_t
		{
			RES_OK      =  0,
			RES_NO_ACK  = -1,
			RES_TX_FAIL = -2,
			RES_RX_FAIL = -3
		};
		
		i2c(i2c_t i2c, uint32_t baud, dma &dma_tx, dma &dma_rx, gpio &sda,
			gpio &scl);
		~i2c();
		
		void baud(uint32_t baud);
		uint32_t baud() const { return _baud; }
		//int8_t tx(uint16_t addr, void *buff, uint16_t size);
		//int8_t rx(uint16_t addr, void *buff, uint16_t size);
		int8_t exch(uint16_t addr, void *buff_tx, uint16_t size_tx,
			void *buff_rx, uint16_t size_rx);
	
	private:
		i2c_t _i2c;
		uint32_t _baud;
		
		SemaphoreHandle_t api_lock;
		TaskHandle_t task;
		res_t irq_res;
		
		gpio &_sda;
		gpio &_scl;
		
		uint16_t _addr;
		
		dma &tx_dma;
		void *tx_buff;
		uint16_t tx_size;
		
		dma &rx_dma;
		void *rx_buff;
		uint16_t rx_size;
		
		static void gpio_af_init(i2c_t i2c, gpio &gpio);
		static void calc_clk(i2c_t i2c, uint32_t baud, uint8_t &freq,
			uint8_t &trise, uint32_t &ccr);
		static void on_dma_tx(dma *dma, dma::event_t event, void *ctx);
		static void on_dma_rx(dma *dma, dma::event_t event, void *ctx);
		friend void ::tx_hndlr(i2c *obj, BaseType_t *hi_task_woken);
		friend void ::rx_hndlr(i2c *obj, BaseType_t *hi_task_woken);
		friend void ::err_hndlr(i2c *obj, int8_t err, BaseType_t *hi_task_woken);
		friend void ::i2c_event_irq_hndlr(i2c *obj);
		friend void ::i2c_error_irq_hndlr(i2c *obj);
};
}