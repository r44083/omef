#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <algorithm>
#include <climits>
#include "common/assert.h"
#include "spi.hpp"
#include "ESP8266_RTOS_SDK/components/esp8266/include/esp8266/spi_register.h"
#include "ESP8266_RTOS_SDK/components/esp8266/include/esp8266/spi_struct.h"
#include "ESP8266_RTOS_SDK/components/esp8266/include/rom/ets_sys.h"
#include "ESP8266_RTOS_SDK/components/esp8266/include/esp8266/pin_mux_register.h"

using namespace hal;

constexpr uint8_t fifo_size = sizeof(((spi_dev_t *)0)->data_buf);
constexpr uint32_t irq_sta_bits = (SPI_TRANS_DONE | SPI_SLV_WR_STA_DONE |
	SPI_SLV_RD_STA_DONE | SPI_SLV_WR_BUF_DONE | SPI_SLV_RD_BUF_DONE);
constexpr uint32_t irq_ena_bits = (SPI_TRANS_DONE_EN | SPI_SLV_WR_STA_DONE_EN |
	SPI_SLV_RD_STA_DONE_EN | SPI_SLV_WR_BUF_DONE_EN | SPI_SLV_RD_BUF_DONE_EN);

static DRAM_ATTR spi_dev_t *const spi_devs[] = {&SPI0, &SPI1};

static void spi_hw_cb(void *arg);

spi::spi(spi_t spi, uint32_t baud, cpol_t cpol, cpha_t cpha,
	bit_order_t bit_order, gpio &mosi, gpio &miso, gpio &clk):
	_spi(spi),
	_baud(baud),
	_cpol(cpol),
	_cpha(cpha),
	_bit_order(bit_order),
	api_lock(NULL),
	irq_res(RES_OK),
	_mosi(mosi),
	_miso(miso),
	_clk(clk),
	_cs(NULL),
	tx_buff(NULL),
	rx_buff(NULL),
	remain(0)
{
	ASSERT(_spi <= SPI_1);
	ASSERT(_baud > 0 && _baud <= APB_CLK_FREQ);
	ASSERT(_cpol <= CPOL_1);
	ASSERT(_cpha <= CPHA_1);
	ASSERT(api_lock = xSemaphoreCreateMutex());
	
	spi_dev_t &spi_dev = *spi_devs[_spi];
	
	// Master mode
	spi_dev.pin.slave_mode = false;
	spi_dev.slave.slave_mode = false;
	spi_dev.user.usr_mosi_highpart = false;
	spi_dev.user.usr_miso_highpart = false;
	
	spi_dev.user.usr_command = false;
	spi_dev.user.usr_addr = false;
	spi_dev.user.usr_dummy = false;
	
	spi_dev.user.flash_mode = false;
	spi_dev.user.cs_hold = false;
	spi_dev.user.cs_setup = false;
	spi_dev.user.duplex = true;
	spi_dev.user.ck_i_edge = false;
	spi_dev.ctrl2.mosi_delay_num = 0;
	spi_dev.ctrl2.miso_delay_num = 1;
	
	spi_dev.user.fwrite_dual = false;
	spi_dev.user.fwrite_quad = false;
	spi_dev.user.fwrite_dio = false;
	spi_dev.user.fwrite_qio = false;
	spi_dev.ctrl.fread_dual = false;
	spi_dev.ctrl.fread_quad = false;
	spi_dev.ctrl.fread_dio = false;
	spi_dev.ctrl.fread_qio = false;
	
	spi_dev.pin.ck_idle_edge = _cpol == CPOL_1;
	spi_dev.user.ck_out_edge = _cpha == CPHA_1;
	
	spi_dev.ctrl.wr_bit_order = _bit_order == BIT_ORDER_LSB;
	spi_dev.ctrl.rd_bit_order = _bit_order == BIT_ORDER_LSB;
	spi_dev.user.wr_byte_order = false;
	spi_dev.user.rd_byte_order = false;
	
	spi_div_t div;
	uint16_t prediv;
	calc_clk(_spi, _baud, div, prediv);
	
	if(div == SPI_DIV_80MHz)
	{
		SET_PERI_REG_MASK(PERIPHS_IO_MUX_CONF_U, _spi == SPI_0 ?
			SPI0_CLK_EQU_SYS_CLK : SPI1_CLK_EQU_SYS_CLK);
		spi_dev.clock.clk_equ_sysclk = true;
	}
	else
	{
		CLEAR_PERI_REG_MASK(PERIPHS_IO_MUX_CONF_U, _spi == SPI_0 ?
			SPI0_CLK_EQU_SYS_CLK : SPI1_CLK_EQU_SYS_CLK);
		spi_dev.clock.clk_equ_sysclk = false;
		spi_dev.clock.clkdiv_pre = prediv;
		spi_dev.clock.clkcnt_n = div - 1;
		spi_dev.clock.clkcnt_l = div - 1;
		spi_dev.clock.clkcnt_h = (div / 2) - 1;
	}
	
	/* SPI0 irq is enabled by default. It leads to false positives irq handler
	calls. So disable it if SPI0 isn't initialized explicitly */
	handle_spi0_enabled_irq(_spi);
	
	_xt_isr_attach(ETS_SPI_INUM, spi_hw_cb, this);
	_xt_isr_unmask(1 << ETS_SPI_INUM);
}

spi::~spi()
{
	_xt_isr_mask(1 << ETS_SPI_INUM);
	_xt_isr_attach(ETS_SPI_INUM, NULL, NULL);
	spi_devs[_spi]->slave.val &= ~(irq_ena_bits | irq_sta_bits);
	xSemaphoreGive(api_lock);
	vSemaphoreDelete(api_lock);
}

void spi::baud(uint32_t baud)
{
	ASSERT(baud > 0 && baud <= APB_CLK_FREQ);
	
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	_baud = baud;
	
	spi_div_t div;
	uint16_t prediv;
	calc_clk(_spi, _baud, div, prediv);
	
	if(div == SPI_DIV_80MHz)
	{
		SET_PERI_REG_MASK(PERIPHS_IO_MUX_CONF_U, _spi == SPI_0 ?
			SPI0_CLK_EQU_SYS_CLK : SPI1_CLK_EQU_SYS_CLK);
		spi_devs[_spi]->clock.clk_equ_sysclk = true;
	}
	else
	{
		CLEAR_PERI_REG_MASK(PERIPHS_IO_MUX_CONF_U, _spi == SPI_0 ?
			SPI0_CLK_EQU_SYS_CLK : SPI1_CLK_EQU_SYS_CLK);
		spi_dev_t &spi_dev = *spi_devs[_spi];
		spi_dev.clock.clk_equ_sysclk = false;
		spi_dev.clock.clkdiv_pre = prediv;
		spi_dev.clock.clkcnt_n = div - 1;
		spi_dev.clock.clkcnt_l = div - 1;
		spi_dev.clock.clkcnt_h = (div / 2) - 1;
	}
	
	xSemaphoreGive(api_lock);
}

void spi::cpol(cpol_t cpol)
{
	ASSERT(cpol <= CPOL_1);
	
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	_cpol = cpol;
	spi_devs[_spi]->pin.ck_idle_edge = _cpol == CPOL_1;
	
	xSemaphoreGive(api_lock);
}

void spi::cpha(cpha_t cpha)
{
	ASSERT(cpha <= CPHA_1);
	
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	_cpha = cpha;
	spi_devs[_spi]->user.ck_out_edge = _cpha == CPHA_1;
	
	xSemaphoreGive(api_lock);
}

void spi::bit_order(bit_order_t bit_order)
{
	ASSERT(bit_order <= BIT_ORDER_LSB);
	
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	_bit_order = bit_order;
	
	spi_devs[_spi]->ctrl.wr_bit_order = _bit_order == BIT_ORDER_LSB;
	spi_devs[_spi]->ctrl.rd_bit_order = _bit_order == BIT_ORDER_LSB;
	
	xSemaphoreGive(api_lock);
}

int8_t spi::write(void *buff, size_t size, gpio *cs)
{
	ASSERT(buff);
	ASSERT(size > 0);
	
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	_cs = cs;
	if(_cs)
		_cs->set(0);
	
	task = xTaskGetCurrentTaskHandle();
	tx_buff = (uint8_t *)buff;
	rx_buff = NULL;
	remain = size;
	uint8_t payload_size = std::min<size_t>(size, fifo_size);
	
	spi_dev_t &spi_dev = *spi_devs[_spi];
	spi_dev.user.usr_mosi = true;
	spi_dev.user.usr_miso = false;
	spi_dev.user1.usr_mosi_bitlen = (payload_size * CHAR_BIT) - 1;
	memcpy((void *)spi_dev.data_buf, tx_buff, payload_size);
	tx_buff += payload_size;
	
	portENTER_CRITICAL();
	spi_dev.cmd.usr = 1;
	spi_dev.slave.trans_done = false;
	spi_dev.slave.trans_inten = true;
	portEXIT_CRITICAL();
	
	// Task will be unlocked later from isr
	ulTaskNotifyTake(true, portMAX_DELAY);
	
	xSemaphoreGive(api_lock);
	
	return irq_res;
}

int8_t spi::write(uint8_t byte, gpio *cs)
{
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	_cs = cs;
	if(_cs)
		_cs->set(0);
	
	task = xTaskGetCurrentTaskHandle();
	tx_buff = NULL;
	rx_buff = NULL;
	remain = sizeof(byte);
	
	spi_dev_t &spi_dev = *spi_devs[_spi];
	spi_dev.user.usr_mosi = true;
	spi_dev.user.usr_miso = false;
	spi_dev.user1.usr_mosi_bitlen = CHAR_BIT - 1;
	spi_dev.data_buf[0] = byte;
	
	portENTER_CRITICAL();
	spi_dev.cmd.usr = 1;
	spi_dev.slave.trans_done = false;
	spi_dev.slave.trans_inten = true;
	portEXIT_CRITICAL();
	
	// Task will be unlocked later from isr
	ulTaskNotifyTake(true, portMAX_DELAY);
	
	xSemaphoreGive(api_lock);
	
	return irq_res;
}

int8_t spi::read(void *buff, size_t size, gpio *cs)
{
	ASSERT(buff);
	ASSERT(size > 0);
	
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	_cs = cs;
	if(_cs)
		_cs->set(0);
	
	task = xTaskGetCurrentTaskHandle();
	rx_buff = (uint8_t *)buff;
	tx_buff = NULL;
	remain = size;
	uint8_t payload_size = std::min<size_t>(size, fifo_size);
	
	spi_dev_t &spi_dev = *spi_devs[_spi];
	spi_dev.user.usr_mosi = false;
	spi_dev.user.usr_miso = true;
	spi_dev.user1.usr_miso_bitlen = (payload_size * CHAR_BIT) - 1;
	
	portENTER_CRITICAL();
	spi_dev.cmd.usr = 1;
	spi_dev.slave.trans_done = false;
	spi_dev.slave.trans_inten = true;
	portEXIT_CRITICAL();
	
	// Task will be unlocked later from isr
	ulTaskNotifyTake(true, portMAX_DELAY);
	
	xSemaphoreGive(api_lock);
	
	return irq_res;
}

int8_t spi::exch(void *buff_tx, void *buff_rx, size_t size, gpio *cs)
{
	ASSERT(buff_tx);
	ASSERT(buff_rx);
	ASSERT(size > 0);
	
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	_cs = cs;
	if(_cs)
		_cs->set(0);
	
	task = xTaskGetCurrentTaskHandle();
	tx_buff = (uint8_t *)buff_tx;
	rx_buff = (uint8_t *)buff_rx;
	remain = size;
	uint8_t payload_size = std::min<size_t>(size, fifo_size);
	
	spi_dev_t &spi_dev = *spi_devs[_spi];
	spi_dev.user.usr_mosi = true;
	spi_dev.user.usr_miso = true;
	spi_dev.user1.usr_mosi_bitlen = (payload_size * CHAR_BIT) - 1;
	spi_dev.user1.usr_miso_bitlen = (payload_size * CHAR_BIT) - 1;
	memcpy((void *)spi_dev.data_buf, tx_buff, payload_size);
	tx_buff += payload_size;
	
	portENTER_CRITICAL();
	spi_dev.cmd.usr = 1;
	spi_dev.slave.trans_done = false;
	spi_dev.slave.trans_inten = true;
	portEXIT_CRITICAL();
	
	// Task will be unlocked later from isr
	ulTaskNotifyTake(true, portMAX_DELAY);
	
	xSemaphoreGive(api_lock);
	
	return irq_res;
}

void spi::calc_clk(spi_t spi, uint32_t baud, spi_div_t &div, uint16_t &prediv)
{
	div = SPI_DIV_2MHz;
	prediv = 0;
}

void spi::handle_spi0_enabled_irq(spi_t spi)
{
	static bool is_spi0_initialized = false;
	if(spi == SPI_0)
		is_spi0_initialized = true;
	else if(!is_spi0_initialized && (SPI0.slave.val & irq_ena_bits))
		SPI0.slave.val &= ~irq_ena_bits;
}

extern "C" void spi_irq_hndlr(hal::spi *obj)
{
	spi_dev_t &spi_dev = *spi_devs[obj->_spi];
	spi_dev.slave.trans_done = false;
	
	uint8_t last_payload = std::min<size_t>(obj->remain, fifo_size);
	obj->remain -= last_payload;
	uint8_t new_payload = std::min<size_t>(obj->remain, fifo_size);
	
	if(obj->rx_buff)
	{
		memcpy(obj->rx_buff, (void *)spi_dev.data_buf, last_payload);
		obj->rx_buff += last_payload;
	}
	
	if(new_payload)
	{
		if(obj->rx_buff)
			spi_dev.user1.usr_miso_bitlen = (new_payload * CHAR_BIT) - 1;
		
		if(obj->tx_buff)
		{
			spi_dev.user1.usr_mosi_bitlen = (new_payload * CHAR_BIT) - 1;
			memcpy((void *)spi_dev.data_buf, obj->tx_buff, new_payload);
			obj->tx_buff += new_payload;
		}
		spi_dev.cmd.usr = 1;
		return;
	}
	
	spi_dev.slave.trans_inten = false;
	if(obj->_cs)
	{
		obj->_cs->set(1);
		obj->_cs = NULL;
	}
	
	obj->irq_res = spi::RES_OK;
	
	BaseType_t hi_task_woken = 0;
	vTaskNotifyGiveFromISR(obj->task, &hi_task_woken);
	if(hi_task_woken)
		taskYIELD();
}

static void spi_hw_cb(void *arg)
{
#define SPI_INT_STATUS_REG 0x3FF00020
#define SPI_INT_STATUS_SPI0_SEL (1 << 4)
#define SPI_INT_STATUS_SPI1_SEL (1 << 7)
#define SPI_INT_STATUS_I2S_SEL (1 << 9)
	
	// Don't handle I2S interrupt here
	if(READ_PERI_REG(SPI_INT_STATUS_REG) & SPI_INT_STATUS_I2S_SEL)
		return;
	
	spi_irq_hndlr((spi *)arg);
}