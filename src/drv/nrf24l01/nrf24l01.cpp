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
	hal::tim &tim, dev dev):
	_spi(spi),
	_cs(cs),
	_ce(ce),
	_exti(exti),
	_tim(tim),
	_conf({.dev = dev})
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
	
	int8_t res = set_mode(mode::PWR_DOWN);
	if(res)
		return res;
	
	setup_aw_reg_t setup_aw;
	if((res = read_reg(reg::SETUP_AW, &setup_aw)))
		return res;
	// Check for invalid response from nrf24l01
	if(setup_aw.aw == 0 || setup_aw.reserved != 0)
		return RES_NRF_NO_RESPONSE_ERR;
	
	if(_conf.dev == dev::NRF24L01_PLUS)
	{
		feature_reg_t feature;
		if((res = read_reg(reg::CONFIG, &feature)))
			return res;
		_conf.dyn_payload = feature.en_dpl;
		_conf.ack_payload = feature.en_ack_pay && feature.en_dpl;
	}
	
	config_reg_t config = {.crco = _2_BYTE, .en_crc = 1};
	if((res = write_reg(reg::CONFIG, &config)))
		return res;
	
	// Disable all rx data pipes, since pipe 0 and 1 are enabled by default
	uint8_t en_rxaddr = 0;
	if((res = write_reg(reg::EN_RXADDR, &en_rxaddr)))
		return res;
	memset(_conf.pipe, 0, sizeof(_conf.pipe));
	
	if((res = exec_instruction(instruction::FLUSH_TX)))
		return res;
	
	if((res = exec_instruction(instruction::FLUSH_RX)))
		return res;
	
	// Reset IRQ flags
	status_reg_t status = {.max_rt = 1, .tx_ds = 1, .rx_dr = 1};
	if((res = write_reg(reg::STATUS, &status)))
		return res;
	
	return res;
}

int8_t nrf24l01::get_conf(conf_t &conf)
{
	freertos::semaphore_take(api_lock, portMAX_DELAY);
	
	memset(&conf, 0, sizeof(conf));
	
	int8_t res;
	uint8_t en_rxaddr;
	if((res = read_reg(reg::EN_RXADDR, &en_rxaddr)))
		return res;
	
	uint8_t en_aa;
	if((res = read_reg(reg::EN_AA, &en_aa)))
		return res;
	
	uint8_t dynpd;
	if(_conf.dev == dev::NRF24L01_PLUS)
	{
		if((res = read_reg(reg::DYNPD, &dynpd)))
			return res;
		
		feature_reg_t feature;
		if((res = read_reg(reg::FEATURE, &feature)))
			return res;
		conf.dyn_payload = feature.en_dpl;
	}
	
	for(uint8_t i = 0; i < pipes; i++)
	{
		conf.pipe[i].enable = en_rxaddr & (1 << i);
		
		uint8_t rx_pw = static_cast<uint8_t>(reg::RX_PW_P0) + i;
		uint8_t rx_pw_val = 0;
		if((res = read_reg(static_cast<enum reg>(rx_pw), &rx_pw_val)))
			return res;
		conf.pipe[i].size = rx_pw_val;
		_conf.pipe[i].size = rx_pw_val;
		
		uint8_t rx_addr = static_cast<uint8_t>(reg::RX_ADDR_P0) + i;
		uint64_t rx_addr_val = 0;
		if((res = read_reg(static_cast<enum reg>(rx_addr), &rx_addr_val,
			i < 2 ? addr_max_size : addr_min_size)))
		{
			return res;
		}
		conf.pipe[i].addr = rx_addr_val;
		
		conf.pipe[i].auto_ack = en_aa & (1 << i);
		
		if(_conf.dev == dev::NRF24L01_PLUS)
		{
			conf.pipe[i].dyn_payload = dynpd & (1 << i);
			_conf.pipe[i].dyn_payload = dynpd & (1 << i);
		}
	}
	
	uint64_t tx_addr_val = 0;
	if((res = read_reg(reg::TX_ADDR, &tx_addr_val, addr_max_size)))
		return res;
	conf.tx_addr = tx_addr_val;
	
	conf.tx_auto_ack = (conf.tx_addr == conf.pipe[0].addr &&
		conf.pipe[0].enable/* && conf.pipe[0].size == fifo_size*/);
	
	return res;
}

int8_t nrf24l01::set_conf(conf_t &conf)
{
	ASSERT(is_conf_valid(conf));
	
	freertos::semaphore_take(api_lock, portMAX_DELAY);
	
	if(conf.tx_auto_ack)
	{
		conf.pipe[0].enable = true;
		conf.pipe[0].size = fifo_size;
		conf.pipe[0].addr = conf.tx_addr;
		conf.pipe[0].auto_ack = true;
		conf.pipe[0].dyn_payload = conf.dyn_payload;
	}
	
	int8_t res;
	uint8_t en_rxaddr = 0, en_aa = 0, dynpd = 0;
	for(uint8_t i = 0; i < pipes; i++)
	{
		en_rxaddr |= conf.pipe[i].enable << i;
		en_aa |= conf.pipe[i].auto_ack << i;
		if(_conf.dev == dev::NRF24L01_PLUS)
		{
			dynpd |= conf.pipe[i].dyn_payload << i;
			_conf.pipe[i].dyn_payload = conf.pipe[i].dyn_payload;
		}
		
		uint8_t rx_pw = static_cast<uint8_t>(reg::RX_PW_P0) + i;
		uint8_t rx_pw_val = conf.pipe[i].size;
		if((res = write_reg(static_cast<enum reg>(rx_pw), &rx_pw_val)))
			return res;
		_conf.pipe[i].size = conf.pipe[i].size;
		
		uint8_t rx_addr = static_cast<uint8_t>(reg::RX_ADDR_P0) + i;
		uint64_t rx_addr_val = conf.pipe[i].addr;
		if((res = write_reg(static_cast<enum reg>(rx_addr), &rx_addr_val,
			i < 2 ? addr_max_size : addr_min_size)))
		{
			return res;
		}
	}
	
	if((res = write_reg(reg::EN_RXADDR, &en_rxaddr)))
		return res;
	
	if((res = write_reg(reg::EN_AA, &en_aa)))
		return res;
	
	uint64_t tx_addr_val = conf.tx_addr;
	if((res = write_reg(reg::TX_ADDR, &tx_addr_val, addr_max_size)))
		return res;
	
	if(_conf.dev == dev::NRF24L01_PLUS)
	{
		if((res = write_reg(reg::DYNPD, &dynpd)))
			return res;
		
		feature_reg_t feature;
		if((res = read_reg(reg::FEATURE, &feature)))
			return res;
		
		if(feature.en_dpl != conf.dyn_payload)
		{
			feature.en_dpl = conf.dyn_payload;
			if((res = write_reg(reg::FEATURE, &feature)))
				return res;
		}
		_conf.dyn_payload = feature.en_dpl;
		_conf.ack_payload = feature.en_dpl && feature.en_ack_pay;
	}
	
	if((res = exec_instruction(instruction::FLUSH_TX)))
		return RES_SPI_ERR;
	
	if((res = exec_instruction(instruction::FLUSH_RX)))
		return RES_SPI_ERR;
	
	if(!en_rxaddr && _conf.mode != mode::PWR_DOWN)
		res = set_mode(mode::PWR_DOWN);
	
	return res;
}

bool nrf24l01::is_conf_valid(conf_t &conf)
{
	// Pipe max addr length = 5 bytes
	if(conf.tx_addr > 0xFFFFFFFFFF)
		return false;
	
	if(_conf.dev != dev::NRF24L01_PLUS && conf.dyn_payload)
		return false;
	
	for(uint8_t i = 0; i < pipes; i++)
	{
		if(conf.pipe[i].size > fifo_size)
			return false;
		
		// Pipe max addr length = 5 bytes
		if(conf.pipe[i].addr > 0xFFFFFFFFFF)
			return false;
		
		// Only byte 0 can be configured in pipe rx address for pipe number 2-5
		if(i > 1 && conf.pipe[i].addr > 0xFF)
			return false;
		
		if(_conf.dev != dev::NRF24L01_PLUS && conf.pipe[i].dyn_payload)
			return false;
	}
	return true;
}

int8_t nrf24l01::read(rx_packet_t &packet, ack_payload_t *ack)
{
	/* If ack payload is provided, dynamic payload size should be configured and
	ack payload size must be also valid */
	ASSERT(!ack || (_conf.dyn_payload && _conf.dev == dev::NRF24L01_PLUS &&
		ack->size > 0 && ack->size <= fifo_size));
	
	freertos::semaphore_take(api_lock, portMAX_DELAY);
	
	// Check FIFO_STATUS first to read the data received earlier is exist
	int8_t res;
	fifo_status_reg_t fifo_status;
	if((res = read_reg(reg::FIFO_STATUS, &fifo_status)))
		return res;
	
	if(fifo_status.rx_empty)
	{
		if(_conf.mode != mode::RX)
		{
			if((res = set_mode(mode::RX)))
				return res;
		}
		
		if(static_cast<bool>(ack) != _conf.ack_payload)
		{
			if((res = ack_payload(static_cast<bool>(ack))))
				return res;
		}
		if(ack)
		{
			// Write ack payload to be transmitted after reception
			if((res = exec_instruction_with_write(instruction::W_ACK_PAYLOAD,
				ack->buff, ack->size)))
			{
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
	if((res = read_reg(reg::STATUS, &status)))
		goto exit;
	
	// TODO: is it needed to check status.rx_p_no for valid range?
	if(_conf.pipe[status.rx_p_no].dyn_payload)
	{
		if((res = exec_instruction_with_read(instruction::R_RX_PL_WID,
			&packet.size, sizeof(packet.size))))
		{
			goto exit;
		}
	}
	else
		packet.size = _conf.pipe[status.rx_p_no].size;
	
	res = exec_instruction_with_read(instruction::R_RX_PAYLOAD, packet.buff,
		packet.size);
	packet.pipe = status.rx_p_no;
	
exit:
	status.rx_dr = 1;
	status.tx_ds = 1;
	status.max_rt = 1;
	int8_t res_final = write_reg(reg::STATUS, &status);
	return res ? res : res_final;
}

int8_t nrf24l01::write(void *buff, size_t size, ack_payload_t *ack,
	bool is_continuous_tx)
{
	ASSERT(buff);
	ASSERT(size > 0 && size <= fifo_size);
	// Ack payload supported only for nrf24l01+
	ASSERT(!ack || (_conf.dyn_payload && _conf.dev == dev::NRF24L01_PLUS));
	
	freertos::semaphore_take(api_lock, portMAX_DELAY);
	
	int8_t res;
	
	if(_conf.mode != mode::TX)
	{
		if((res = set_mode(mode::TX)))
		{
			_ce.set(0);
			return res;
		}
	}
	
	if(static_cast<bool>(ack) != _conf.ack_payload)
	{
		if((res = ack_payload(static_cast<bool>(ack))))
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
		if(is_continuous_tx)
			_ce.set(0);
		_exti.off();
		return RES_NRF_NO_RESPONSE_ERR;
	}
	_exti.off();
	
	status_reg_t status;
	if((res = read_reg(reg::STATUS, &status)))
	{
		if(is_continuous_tx)
			_ce.set(0);
		memset(&status, 0, sizeof(status));
	}
	
	int8_t txrx_res = RES_OK;
	if(status.max_rt)
	{
		// Tx fifo payload isn't removed in case of max_rt. So flush it manually
		exec_instruction(instruction::FLUSH_TX);
		txrx_res = RES_TX_MAX_RETRIES_ERR;
	}
	else if(status.rx_dr) // Payload was received
	{
		if(ack)
		{
			// Get ack payload length and read it
			txrx_res = exec_instruction_with_read(instruction::R_RX_PL_WID,
				&ack->size, sizeof(ack->size));
			
			if(!txrx_res)
			{
				txrx_res = exec_instruction_with_read(instruction::R_RX_PAYLOAD,
					ack->buff, ack->size);
			}
			else
				exec_instruction(instruction::FLUSH_RX);
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
			if(_conf.mode == mode::PWR_DOWN)
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
	
	_conf.mode = mode;
	return RES_OK;
}

int8_t nrf24l01::ack_payload(bool enable)
{
	int8_t res;
	feature_reg_t feature;
	if((res = read_reg(reg::FEATURE, &feature)))
		return res;
	
	if((feature.en_dpl != enable) || (feature.en_ack_pay != enable))
	{
		/* Do not turn-off dynamic payload size if it was explicitly enabled by
		user */
		feature.en_dpl = enable || _conf.dyn_payload;
		
		feature.en_ack_pay = enable;
		
		if((res = write_reg(reg::FEATURE, &feature)))
			return res;
	}
	_conf.ack_payload = enable;
	
	// Setup dynamic payload for pipe0 which is used for tx
	if(_conf.mode == mode::TX)
	{
		uint8_t dynpd;
		if((res = read_reg(reg::DYNPD, &dynpd)))
			return res;

		if(enable != (dynpd & 1))
		{
			if(enable)
				dynpd |= 1;
			else
				dynpd &= ~1;
			
			if((res = write_reg(reg::DYNPD, &dynpd)))
				return res;
		}
	}
	
	return res;
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