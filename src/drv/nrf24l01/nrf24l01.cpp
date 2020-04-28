#include <stddef.h>
#include <string.h>
#include "common/assert.h"
#include "common/freertos_wrappers.hpp"
#include "nrf24l01.hpp"
#include "nrf24l01_priv.hpp"
#include "FreeRTOS.h"
#include "task.h"

using namespace drv;
using namespace nrf24l01_priv;

nrf24l01::nrf24l01(hal::spi &spi, hal::gpio &cs, hal::gpio &ce, hal::exti &exti,
	hal::tim &tim):
	_spi(spi),
	_cs(cs),
	_ce(ce),
	_exti(exti),
	_tim(tim),
	conf({})
{
	ASSERT(_spi.cpol() == hal::spi::CPOL_0);
	ASSERT(_spi.cpha() == hal::spi::CPHA_0);
	ASSERT(_spi.bit_order() == hal::spi::BIT_ORDER_MSB);
	ASSERT(_cs.mode() == hal::gpio::mode::DO);
	ASSERT(_ce.mode() == hal::gpio::mode::DO);
	
	_cs.set(1);
	_ce.set(0);
	
	ASSERT(api_lock = xSemaphoreCreateMutex());
}

nrf24l01::~nrf24l01()
{
	// Reset IRQ flags
	status_reg_t status = {.max_rt = 1, .tx_ds = 1, .rx_dr = 1};
	write_reg(reg::STATUS, &status);
	
	_cs.set(1);
	_ce.set(0);
	_exti.off();
	_tim.stop();
	xSemaphoreGive(api_lock);
	vSemaphoreDelete(api_lock);
}

int8_t nrf24l01::init()
{
	freertos::semaphore_take(api_lock, portMAX_DELAY);
	
	_ce.set(0);
	vTaskDelay(power_on_reset_timeout);
	int8_t res;
	setup_aw_reg_t setup_aw;
	if((res = read_reg(reg::SETUP_AW, &setup_aw)))
		return res;
	// Check for invalid response from nrf24l01
	if(setup_aw.aw == 0 || setup_aw.reserved != 0)
		return RES_NRF_NO_RESPONSE_ERR;
	
	config_reg_t config = {.crco = _2_BYTE, .en_crc = 1};
	if((res = write_reg(reg::CONFIG, &config)))
		return res;
	conf.mode = mode::PWR_DOWN;
	
	// Disable all rx data pipes, since pipe 0 and 1 are enabled by default
	uint8_t en_rxaddr = 0;
	if((res = write_reg(reg::EN_RXADDR, &en_rxaddr)))
		return res;
	memset(conf.pipe, 0, sizeof(conf.pipe));
	
	// Reset IRQ flags
	status_reg_t status = {.max_rt = 1, .tx_ds = 1, .rx_dr = 1};
	if((res = write_reg(reg::STATUS, &status)))
		return res;
	
	if((res = exec_instruction(instruction::FLUSH_TX)))
		return RES_SPI_ERR;
	
	if((res = exec_instruction(instruction::FLUSH_RX)))
		return RES_SPI_ERR;
	
	return res;
}

int8_t nrf24l01::open_pipe(uint8_t pipe, uint64_t addr, uint8_t size,
	bool auto_ack)
{
	ASSERT(pipe < pipes);
	ASSERT(addr <= 0xFFFFFFFFFF);
	// Only byte 0 can be configured in pipe rx address for pipe number 2-5
	ASSERT(pipe < 2 || addr <= 0xFF);
	ASSERT(size > 0 && size <= fifo_size);
	
	freertos::semaphore_take(api_lock, portMAX_DELAY);
	
	// Set pipe address
	int8_t res;
	uint8_t reg = static_cast<uint8_t>(reg::RX_ADDR_P0) + pipe;
	if((res = write_reg(static_cast<enum reg>(reg), &addr,
		pipe < 2 ? addr_max_size : addr_min_size)))
	{
		return res;
	}
	
	// Set pipe payload size
	reg = static_cast<uint8_t>(reg::RX_PW_P0) + pipe;
	if((res = write_reg(static_cast<enum reg>(reg), &size)))
		return res;
	conf.pipe[pipe].size = size;
	
	// Set auto acknowledgmentg
	uint8_t en_aa;
	if((res = read_reg(reg::EN_AA, &en_aa)))
		return res;
	
	if(auto_ack)
		en_aa |= 1 << pipe;
	else
		en_aa &= ~(1 << pipe);
	
	if((res = write_reg(reg::EN_AA, &en_aa)))
		return RES_SPI_ERR;
	
	// Enable data pipe
	uint8_t en_rxaddr;
	if((res = read_reg(reg::EN_RXADDR, &en_rxaddr)))
		return res;
	
	en_rxaddr |= 1 << pipe;
	if((res = write_reg(reg::EN_RXADDR, &en_rxaddr)))
		return res;
	conf.pipe[pipe].is_open = true;
	
	return res;
}

int8_t nrf24l01::read(rx_packet_t &packet, uint8_t tx_ack[fifo_size])
{
	freertos::semaphore_take(api_lock, portMAX_DELAY);
	
	// Check FIFO_STATUS first to read the data received earlier is exist
	fifo_status_reg_t fifo_status;
	int8_t res;
	if((res = read_reg(reg::FIFO_STATUS, &fifo_status)))
		return res;
	
	if(fifo_status.rx_empty)
	{
		if(conf.mode != mode::RX)
		{
			if((res = set_mode(mode::RX)))
				return res;
		}
		
		if(tx_ack)
		{
			// Write ack payload to be transmitted after reception
			if((res = exec_instruction_with_write(instruction::W_TX_PAYLOAD,
				tx_ack, fifo_size)))
			{
				_ce.set(0);
				return res;
			}
		}
		
		// Wait for data
		_exti.cb(exti_cb, xTaskGetCurrentTaskHandle());
		_ce.set(1);
		_exti.on();
		ulTaskNotifyTake(true, portMAX_DELAY);
		_exti.off();
	}
	
	// Read received data
	status_reg_t status;
	res = read_reg(reg::STATUS, &status);
	int8_t res2 = exec_instruction_with_read(instruction::R_RX_PAYLOAD,
		packet.buff, fifo_size);
	/* Remember new error code if it isn't OK, since we can't return it now
	(need to reset IRQ by writing updated spi status register) */
	res = res ? res : res2;
	
	status.rx_dr = 1;
	status.tx_ds = 1;
	status.max_rt = 1;
	res2 = write_reg(reg::STATUS, &status);
	
	packet.pipe = status.rx_p_no;
	packet.size = conf.pipe[packet.pipe].size;
	
	return res ? res : res2;
}

int8_t nrf24l01::close_pipe(uint8_t pipe)
{
	ASSERT(pipe < pipes);
	
	freertos::semaphore_take(api_lock, portMAX_DELAY);
	
	// Disable data pipe
	uint8_t en_rxaddr;
	int8_t res;
	if((res = read_reg(reg::EN_RXADDR, &en_rxaddr)))
		return res;
	
	en_rxaddr &= ~(1 << pipe);
	if((res = write_reg(reg::EN_RXADDR, &en_rxaddr)))
		return res;
	conf.pipe[pipe].is_open = false;
	
	bool is_any_pipe_open = false;
	for(uint8_t i = 0; i < pipes; i++)
	{
		if(conf.pipe[i].is_open)
		{
			is_any_pipe_open = true;
			break;
		}
	}
	if(!is_any_pipe_open)
		res = set_mode(mode::PWR_DOWN);
	
	return res;
}

int8_t nrf24l01::tx_addr(uint64_t addr)
{
	ASSERT(addr <= 0xFFFFFFFFFF);
	
	freertos::semaphore_take(api_lock, portMAX_DELAY);
	
	// Set tx address
	int8_t res;
	if((res = write_reg(reg::TX_ADDR, &addr, addr_max_size)))
		return res;
	
	/* Set RX_ADDR_P0 equal to TX_ADDR address to handle automatic acknowledge
	   if this is a PTX device with Enhanced ShockBurst enabled. See page 14.
	*/
	if((res = write_reg(reg::RX_ADDR_P0, &addr, addr_max_size)))
		return res;
	
	// Enable rx data pipe0 for automatic acknowledge
	uint8_t en_rxaddr;
	if((res = read_reg(reg::EN_RXADDR, &en_rxaddr)))
		return res;
	
	en_rxaddr |= 1;
	if((res = write_reg(reg::EN_RXADDR, &en_rxaddr)))
		return res;
	conf.pipe[0].size = fifo_size;
	conf.pipe[0].is_open = true;
	
	if((res = exec_instruction(instruction::FLUSH_TX)))
		return res;
	
	return res;
}

int8_t nrf24l01::write(void *buff, size_t size, void *ack_payload,
	bool is_continuous_tx)
{
	ASSERT(buff);
	ASSERT(size > 0 && size <= fifo_size);
	
	freertos::semaphore_take(api_lock, portMAX_DELAY);
	
	int8_t res;
	
	if(conf.mode != mode::TX)
	{
		if((res = set_mode(mode::TX)))
		{
			_ce.set(0);
			return res;
		}
	}
	
	if((res = exec_instruction_with_write(instruction::W_TX_PAYLOAD, buff,
		size)))
	{
		_ce.set(0);
		return res;
	}
	
	_exti.cb(exti_cb, xTaskGetCurrentTaskHandle());
	_exti.on();
	
	_ce.set(1);
	/* is_continuous_tx 0: go to standby-1 mode after transmitting one packet
	   is_continuous_tx 1: go to standby-2 mode when fifo will be empty */
	if(!is_continuous_tx)
	{
		delay(10);
		_ce.set(0);
	}
	
	// TODO: Calculate presize timeout based on values of arc and ard
	if(!ulTaskNotifyTake(true, transmit_max_timeout))
	{
		_ce.set(0);
		_exti.off();
		return RES_NRF_NO_RESPONSE_ERR;
	}
	_exti.off();
	
	status_reg_t status;
	if((res = read_reg(reg::STATUS, &status)))
		_ce.set(0);
	
	int8_t txrx_res = RES_OK;
	if(status.max_rt)
	{
		// Tx fifo payload isn't removed in case of max_rt. So flush it manually
		exec_instruction(instruction::FLUSH_TX);
		txrx_res = RES_TX_MAX_RETRIES_ERR;
	}
	else if(status.rx_dr) // Payload was received
	{
		if(ack_payload)
		{
			txrx_res = exec_instruction_with_read(instruction::R_RX_PAYLOAD,
				ack_payload, size);
		}
		else
			exec_instruction(instruction::FLUSH_RX);
	}
	
	status.max_rt = 1;
	status.tx_ds = 1;
	status.rx_dr = 1;
	res = write_reg(reg::STATUS, &status);
	
	return txrx_res != RES_OK ? txrx_res : res;
}

int8_t nrf24l01::power_down()
{
	freertos::semaphore_take(api_lock, portMAX_DELAY);
	
	return set_mode(mode::PWR_DOWN);
}

int8_t nrf24l01::read_reg(reg reg, void *data, size_t size)
{
	hal::spi_cs cs(_cs);
	
	if(reg == reg::STATUS)
	{
		instruction instruction = instruction::NOP;
		if(_spi.exch(&instruction, data, 1))
			return RES_SPI_ERR;
		
		return RES_OK;
	}
	
	if(_spi.write(static_cast<uint8_t>(instruction::R_REGISTER) |
		static_cast<uint8_t>(reg)))
	{
		return RES_SPI_ERR;
	}
	
	if(_spi.read(data, size))
		return RES_SPI_ERR;
	
	return RES_OK;
}

int8_t nrf24l01::write_reg(reg reg, void *data, size_t size)
{
	hal::spi_cs cs(_cs);
	
	if(_spi.write(static_cast<uint8_t>(instruction::W_REGISTER) |
		static_cast<uint8_t>(reg)))
	{
		return RES_SPI_ERR;
	}
	
	if(_spi.write(data, size))
		return RES_SPI_ERR;
	
	return RES_OK;
}

int8_t nrf24l01::exec_instruction(instruction instruction)
{
	hal::spi_cs cs(_cs);
	
	if(_spi.write(static_cast<uint8_t>(instruction)))
		return RES_SPI_ERR;
	
	return RES_OK;
}

int8_t nrf24l01::exec_instruction_with_read(instruction instruction, void *buff,
	size_t size)
{
	hal::spi_cs cs(_cs);
	
	if(_spi.write(static_cast<uint8_t>(instruction)))
		return RES_SPI_ERR;
	
	if(_spi.read(buff, size))
		return RES_SPI_ERR;
	
	return RES_OK;
}

int8_t nrf24l01::exec_instruction_with_write(instruction instruction,
	void *buff, size_t size)
{
	hal::spi_cs cs(_cs);
	
	if(_spi.write(static_cast<uint8_t>(instruction)))
		return RES_SPI_ERR;
	
	if(_spi.read(buff, size))
		return RES_SPI_ERR;
	
	return RES_OK;
}

int8_t nrf24l01::set_mode(mode mode)
{
	int8_t res;
	config_reg_t config;
	uint32_t us_wait = 0;
	
	switch(mode)
	{
		case mode::PWR_DOWN:
			_ce.set(0);
			if((res = read_reg(reg::CONFIG, &config)))
				return res;
			
			config.pwr_up = false;
			if((res = write_reg(reg::CONFIG, &config)))
				return res;
			break;
		
		case mode::STANDBY_1:
			_ce.set(0);
			if(conf.mode == mode::PWR_DOWN)
			{
				if((res = read_reg(reg::CONFIG, &config)))
					return res;
				
				config.pwr_up = true;
				if((res = write_reg(reg::CONFIG, &config)))
					return res;
				us_wait += powerdown_to_standby1_timeout;
			}
			break;
		
		case mode::RX:
			_ce.set(0); // Disable CE to switch into Standby-1 mode first
			if((res = read_reg(reg::CONFIG, &config)))
				return res;
			
			config.pwr_up = true;
			config.prim_rx = PRX;
			if((res = write_reg(reg::CONFIG, &config)))
				return res;
			/* Don't wait any timeout here since reading is blocking operation.
			Will wait inside read() method */
			
			if((res = exec_instruction(instruction::FLUSH_RX)))
				return RES_SPI_ERR;
			/* Don't set CE=1 here. Manage it in read() method to avoid
			situation when rx IRQ already happened but we haven't had time to
			start waiting for it */
			break;
		
		case mode::TX:
			_ce.set(0); // Disable CE to switch into Standby-1 mode first
			if((res = read_reg(reg::CONFIG, &config)))
				return res;
			if(!config.pwr_up)
			{
				config.pwr_up = true;
				us_wait += powerdown_to_standby1_timeout;
			}
			config.prim_rx = PTX;
			if((res = write_reg(reg::CONFIG, &config)))
				return res;
			us_wait += standby1_to_rxtx_timeout;
			
			if((res = exec_instruction(instruction::FLUSH_TX)))
				return RES_SPI_ERR;
			/* Don't set CE=1 here. Manage CE it in write() method to support
			continuous tx operation */
			break;
	}
	
	if(us_wait)
		delay(us_wait);
	
	conf.mode = mode;
	return RES_OK;
}

void nrf24l01::delay(uint32_t us)
{
	_tim.cb(tim_cb, xTaskGetCurrentTaskHandle());
	_tim.us(us);
	_tim.start();
	ulTaskNotifyTake(true, portMAX_DELAY);
}

void nrf24l01::exti_cb(hal::exti *exti, void *ctx)
{
	TaskHandle_t task = (TaskHandle_t *)ctx;
	
	BaseType_t hi_task_woken = 0;
	vTaskNotifyGiveFromISR(task, &hi_task_woken);
	portYIELD_FROM_ISR(hi_task_woken);
}

void nrf24l01::tim_cb(hal::tim *tim, void *ctx)
{
	TaskHandle_t task = (TaskHandle_t *)ctx;
	
	BaseType_t hi_task_woken = 0;
	vTaskNotifyGiveFromISR(task, &hi_task_woken);
	portYIELD_FROM_ISR(hi_task_woken);
}
