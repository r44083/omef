#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "common/assert.h"
#include "spi.hpp"
#include "spi_priv.hpp"
#include "rcc/rcc.hpp"
#include "CMSIS/Device/STM32F1xx/Include/stm32f1xx.h"
#include "CMSIS/Include/core_cm3.h"

using namespace hal;

static spi *obj_list[spi::SPI_END];

#if configUSE_TRACE_FACILITY
static traceHandle isr_dma_tx, isr_dma_rx, isr_spi;
#endif

spi::spi(spi_t spi, uint32_t baud, cpol_t cpol, cpha_t cpha,
	bit_order_t bit_order, dma &dma_tx, dma &dma_rx, gpio &mosi,
	gpio &miso, gpio &clk):
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
	tx_dma(dma_tx),
	tx_buff(NULL),
	rx_dma(dma_rx),
	rx_buff(NULL)
{
	ASSERT(_spi < SPI_END && spi_priv::spi[_spi]);
	ASSERT(_baud > 0);
	ASSERT(_cpol <= CPOL_1);
	ASSERT(_cpha <= CPHA_1);
	ASSERT(tx_dma.dir() == dma::DIR_MEM_TO_PERIPH);
	ASSERT(tx_dma.inc_size() == dma::INC_SIZE_8);
	ASSERT(rx_dma.dir() == dma::DIR_PERIPH_TO_MEM);
	ASSERT(rx_dma.inc_size() == dma::INC_SIZE_8);
	ASSERT(_mosi.mode() == gpio::mode::AF);
	ASSERT(_miso.mode() == gpio::mode::AF);
	ASSERT(_clk.mode() == gpio::mode::AF);
	
	ASSERT(api_lock = xSemaphoreCreateMutex());
	
#if configUSE_TRACE_FACILITY
	vTraceSetMutexName((void *)api_lock, "spi_api_lock");
	isr_dma_tx = xTraceSetISRProperties("ISR_dma_spi_tx", 1);
	isr_dma_rx = xTraceSetISRProperties("ISR_dma_spi_rx", 1);
	isr_spi = xTraceSetISRProperties("ISR_spi", 1);
#endif
	
	obj_list[_spi] = this;
	
	*spi_priv::rcc_en_reg[_spi] |= spi_priv::rcc_en[_spi];
	*spi_priv::rcc_rst_reg[_spi] |= spi_priv::rcc_rst[_spi];
	*spi_priv::rcc_rst_reg[_spi] &= ~spi_priv::rcc_rst[_spi];
	
	SPI_TypeDef *spi_reg = spi_priv::spi[_spi];
	
	// Master mode
	spi_reg->CR1 |= SPI_CR1_MSTR;
	
	if(_cpol == CPOL_0)
		spi_reg->CR1 &= ~SPI_CR1_CPOL;
	else
		spi_reg->CR1 |= SPI_CR1_CPOL;
	
	if(_cpha == CPHA_0)
		spi_reg->CR1 &= ~SPI_CR1_CPHA;
	else
		spi_reg->CR1 |= SPI_CR1_CPHA;
	
	if(_bit_order == BIT_ORDER_MSB)
		spi_reg->CR1 &= ~SPI_CR1_LSBFIRST;
	else
		spi_reg->CR1 |= SPI_CR1_LSBFIRST;
	
	// Disable NSS hardware management
	spi_reg->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;
	
	uint8_t presc = calc_presc(_spi, _baud);
	spi_reg->CR1 |= ((presc << SPI_CR1_BR_Pos) & SPI_CR1_BR);
	
	spi_reg->CR2 &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
	
	// TODO: overrun error has happend each time with this bit
	//spi_reg->CR2 |= SPI_CR2_ERRIE;
	
	spi_reg->CR1 |= SPI_CR1_SPE;
	
	tx_dma.dst((uint8_t *)&spi_reg->DR);
	rx_dma.src((uint8_t *)&spi_reg->DR);
	
	NVIC_ClearPendingIRQ(spi_priv::irqn[_spi]);
	NVIC_SetPriority(spi_priv::irqn[_spi], 4);
	NVIC_EnableIRQ(spi_priv::irqn[_spi]);
}

spi::~spi()
{
	NVIC_DisableIRQ(spi_priv::irqn[_spi]);
	spi_priv::spi[_spi]->CR1 &= ~SPI_CR1_SPE;
	*spi_priv::rcc_en_reg[_spi] &= ~spi_priv::rcc_en[_spi];
	xSemaphoreGive(api_lock);
	vSemaphoreDelete(api_lock);
	obj_list[_spi] = NULL;
}

void spi::baud(uint32_t baud)
{
	ASSERT(baud > 0);
	
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	_baud = baud;
	uint8_t presc = calc_presc(_spi, _baud);
	
	SPI_TypeDef *spi = spi_priv::spi[_spi];
	
	spi->CR1 &= ~(SPI_CR1_SPE | SPI_CR1_BR);
	spi->CR1 |= ((presc << SPI_CR1_BR_Pos) & SPI_CR1_BR) | SPI_CR1_SPE;
	
	xSemaphoreGive(api_lock);
}

void spi::cpol(cpol_t cpol)
{
	ASSERT(cpol <= CPOL_1);
	
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	_cpol = cpol;
	SPI_TypeDef *spi = spi_priv::spi[_spi];
	
	spi->CR1 &= ~SPI_CR1_SPE;
	
	if(_cpol == CPOL_0)
		spi->CR1 &= ~SPI_CR1_CPOL;
	else
		spi->CR1 |= SPI_CR1_CPOL;
	
	spi->CR1 |= SPI_CR1_SPE;
	
	xSemaphoreGive(api_lock);
}

void spi::cpha(cpha_t cpha)
{
	ASSERT(cpha <= CPHA_1);
	
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	_cpha = cpha;
	SPI_TypeDef *spi = spi_priv::spi[_spi];
	
	spi->CR1 &= ~SPI_CR1_SPE;
	
	if(_cpha == CPHA_0)
		spi->CR1 &= ~SPI_CR1_CPHA;
	else
		spi->CR1 |= SPI_CR1_CPHA;
	
	spi->CR1 |= SPI_CR1_SPE;
	
	xSemaphoreGive(api_lock);
}

void spi::bit_order(bit_order_t bit_order)
{
	ASSERT(bit_order <= BIT_ORDER_LSB);
	
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	_bit_order = bit_order;
	SPI_TypeDef *spi = spi_priv::spi[_spi];
	
	spi->CR1 &= ~SPI_CR1_SPE;
	
	if(_bit_order == BIT_ORDER_MSB)
		spi->CR1 &= ~SPI_CR1_LSBFIRST;
	else
		spi->CR1 |= SPI_CR1_LSBFIRST;
	
	spi->CR1 |= SPI_CR1_SPE;
	
	xSemaphoreGive(api_lock);
}

int8_t spi::write(void *buff, uint16_t size, gpio *cs)
{
	ASSERT(buff);
	ASSERT(size > 0);
	
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	_cs = cs;
	if(_cs)
		_cs->set(0);
	
	task = xTaskGetCurrentTaskHandle();
	tx_buff = buff;
	tx_dma.src((uint8_t*)tx_buff);
	tx_dma.size(size);
	tx_dma.start_once(on_dma_tx, this);
	spi_priv::spi[_spi]->CR2 |= SPI_CR2_TXDMAEN;
	
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
	tx_buff = &byte;
	tx_dma.src((uint8_t*)tx_buff);
	tx_dma.size(1);
	tx_dma.start_once(on_dma_tx, this);
	spi_priv::spi[_spi]->CR2 |= SPI_CR2_TXDMAEN;
	
	// Task will be unlocked later from isr
	ulTaskNotifyTake(true, portMAX_DELAY);
	
	xSemaphoreGive(api_lock);
	return irq_res;
}

int8_t spi::read(void *buff, uint16_t size, gpio *cs)
{
	ASSERT(buff);
	ASSERT(size > 0);
	
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	_cs = cs;
	if(_cs)
		_cs->set(0);
	
	rx_buff = buff;
	rx_dma.dst((uint8_t*)rx_buff);
	rx_dma.size(size);
	rx_dma.start_once(on_dma_rx, this);
	
	task = xTaskGetCurrentTaskHandle();
	// Setup tx for reception
	tx_buff = rx_buff;
	tx_dma.src((uint8_t*)tx_buff);
	tx_dma.size(size);
	tx_dma.start_once(on_dma_tx, this);
	uint8_t dr = spi_priv::spi[_spi]->DR;
	spi_priv::spi[_spi]->CR2 |= SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;
	
	// Task will be unlocked later from isr
	ulTaskNotifyTake(true, portMAX_DELAY);
	
	xSemaphoreGive(api_lock);
	return irq_res;
}

int8_t spi::exch(void *buff_tx, void *buff_rx, uint16_t size, gpio *cs)
{
	ASSERT(buff_tx);
	ASSERT(buff_rx);
	ASSERT(size > 0);
	
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	_cs = cs;
	if(_cs)
		_cs->set(0);
	
	rx_buff = buff_rx;
	rx_dma.dst((uint8_t*)rx_buff);
	rx_dma.size(size);
	uint8_t dr = spi_priv::spi[_spi]->DR;
	
	task = xTaskGetCurrentTaskHandle();
	tx_buff = buff_tx;
	tx_dma.src((uint8_t*)tx_buff);
	tx_dma.size(size);
	rx_dma.start_once(on_dma_rx, this);
	tx_dma.start_once(on_dma_tx, this);
	
	spi_priv::spi[_spi]->CR2 |= SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;
	
	// Task will be unlocked later from isr
	ulTaskNotifyTake(true, portMAX_DELAY);
	
	xSemaphoreGive(api_lock);
	return irq_res;
}

uint8_t spi::calc_presc(spi_t spi, uint32_t baud)
{
	uint32_t div = rcc_get_freq(spi_priv::rcc_src[spi]) / baud;
	
	// Baud rate is too low or too high
	ASSERT(div > 1 && div <= 256);
	
	uint8_t presc = 0;
	// Calculate how many times div can be divided by 2
	while((div /= 2) > 1)
		presc++;
	
	return presc;
}

void spi::on_dma_tx(dma *dma, dma::event_t event, void *ctx)
{
	if(event == dma::EVENT_HALF)
		return;
#if configUSE_TRACE_FACILITY
	vTraceStoreISRBegin(isr_dma_tx);
#endif
	spi *obj = static_cast<spi *>(ctx);
	BaseType_t hi_task_woken = 0;
	
	obj->tx_buff = NULL;
	SPI_TypeDef *spi = spi_priv::spi[obj->_spi];
	
	spi->CR2 &= ~SPI_CR2_TXDMAEN;
	if(event == dma::EVENT_CMPLT)
	{
		if(obj->rx_buff)
		{
#if configUSE_TRACE_FACILITY
			vTraceStoreISREnd(hi_task_woken);
#endif
			return;
		}
		spi->CR2 |= SPI_CR2_TXEIE;
	}
	else if(event == dma::EVENT_ERROR)
	{
		if(obj->rx_buff)
		{
			spi->CR2 &= ~SPI_CR2_RXDMAEN;
			obj->rx_dma.stop();
			obj->rx_buff = NULL;
		}
		
		if(obj->_cs)
		{
			obj->_cs->set(1);
			obj->_cs = NULL;
		}
		
		obj->irq_res = RES_FAIL;
		vTaskNotifyGiveFromISR(obj->task, &hi_task_woken);
		portYIELD_FROM_ISR(hi_task_woken);
	}
	
#if configUSE_TRACE_FACILITY
	vTraceStoreISREnd(hi_task_woken);
#endif
}

void spi::on_dma_rx(dma *dma, dma::event_t event, void *ctx)
{
	if(event == dma::EVENT_HALF)
		return;
#if configUSE_TRACE_FACILITY
	vTraceStoreISRBegin(isr_dma_rx);
#endif
	spi *obj = static_cast<spi *>(ctx);
	SPI_TypeDef *spi = spi_priv::spi[obj->_spi];
	
	obj->rx_buff = NULL;
	spi->CR2 &= ~SPI_CR2_RXDMAEN;
	if(event == dma::EVENT_CMPLT)
		obj->irq_res = RES_OK;
	else if(event == dma::EVENT_ERROR)
		obj->irq_res = RES_FAIL;
	
	if(obj->_cs)
	{
		obj->_cs->set(1);
		obj->_cs = NULL;
	}
	
	BaseType_t hi_task_woken = 0;
	vTaskNotifyGiveFromISR(obj->task, &hi_task_woken);
	portYIELD_FROM_ISR(hi_task_woken);
#if configUSE_TRACE_FACILITY
	vTraceStoreISREnd(hi_task_woken);
#endif
}

extern "C" void spi_irq_hndlr(hal::spi *obj)
{
	SPI_TypeDef *spi = spi_priv::spi[obj->_spi];
	uint32_t sr = spi->SR;
	uint8_t dr = spi->DR;
#if configUSE_TRACE_FACILITY
	vTraceStoreISRBegin(isr_spi);
#endif
	
	if((spi->CR2 & SPI_CR2_TXEIE) && (sr & SPI_SR_TXE))
	{
		spi->CR2 &= ~(SPI_CR2_TXEIE | SPI_CR2_TXDMAEN);
		// Wait for last byte transmission/receiving
		while(spi->SR & SPI_SR_BSY);
		obj->irq_res = spi::RES_OK;
	}
	else if((spi->CR2 & SPI_CR2_ERRIE) &&
		(sr & (SPI_SR_UDR | SPI_SR_MODF | SPI_SR_OVR)))
	{
		spi->CR2 &= ~(SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
		if(obj->tx_buff)
		{
			obj->tx_dma.stop();
			obj->tx_buff = NULL;
		}
		if(obj->rx_buff)
		{
			obj->rx_dma.stop();
			obj->rx_buff = NULL;
		}
		obj->irq_res = spi::RES_FAIL;
	}
	
	if(obj->_cs)
	{
		obj->_cs->set(1);
		obj->_cs = NULL;
	}
	
	BaseType_t hi_task_woken = 0;
	vTaskNotifyGiveFromISR(obj->task, &hi_task_woken);
#if configUSE_TRACE_FACILITY
	vTraceStoreISREnd(hi_task_woken);
#endif
	portYIELD_FROM_ISR(hi_task_woken);
}

extern "C" void SPI1_IRQHandler(void)
{
	spi_irq_hndlr(obj_list[spi::SPI_1]);
}

#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101xB) || \
	defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F102xB) || \
	defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
	defined(STM32F105xC) || defined(STM32F107xC)
extern "C" void SPI2_IRQHandler(void)
{
	spi_irq_hndlr(obj_list[spi::SPI_2]);
}
#endif

#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
	defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
	defined(STM32F107xC)
extern "C" void SPI3_IRQHandler(void)
{
	spi_irq_hndlr(obj_list[spi::SPI_3]);
}
#endif