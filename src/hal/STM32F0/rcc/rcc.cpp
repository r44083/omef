#include <stdint.h>
#include "rcc.hpp"
#include "CMSIS/Device/STM32F0xx/Include/stm32f0xx.h"
#include "CMSIS/Device/STM32F0xx/Include/system_stm32f0xx.h"
#include "CMSIS/Include/core_cm0.h"

extern uint32_t SystemCoreClock;

using namespace hal;

// Converting CFGR[7:4] HPRE value to prescaller
constexpr uint32_t ahb_presc[16] =
{
	0, 0, 0, 0, 0, 0, 0, 0, // 0-7 AHB prescaller 1
	2,                      // 8   AHB prescaller 2
	4,                      // 9   AHB prescaller 4
	8,                      // 10  AHB prescaller 8
	16,                     // 11  AHB prescaller 16
	64,                     // 12  AHB prescaller 64
	128,                    // 13  AHB prescaller 128
	256,                    // 14  AHB prescaller 256
	512                     // 15  AHB prescaller 512
};

// Converting CFGR[10:8] PPRE value to prescaller
constexpr uint32_t apb_presc[8] =
{
	0, 0, 0, 0, // APB prescaller 1
	2,          // APB prescaller 2
	4,          // APB prescaller 4
	8,          // APB prescaller 8
	16          // APB prescaller 16
};

static uint32_t csr = 0;

bool hal::rcc_init(void)
{
	csr = RCC->CSR;
	// Clear the reset reason
	RCC->CSR |= RCC_CSR_RMVF;
	
	// Specific RCC initialization goes here:
	
	return true;
}

uint32_t hal::rcc_get_freq(rcc_src_t src)
{
	uint32_t tmp = (RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos;
	uint32_t _ahb_presc = ahb_presc[tmp];
	
	tmp = (RCC->CFGR & RCC_CFGR_PPRE) >> RCC_CFGR_PPRE_Pos;
	uint32_t _apb_presc = apb_presc[tmp];
	
	SystemCoreClockUpdate();
	uint32_t freq = SystemCoreClock;
	switch(src)
	{
		case RCC_SRC_SYSCLK:
			break;
		
		case RCC_SRC_AHB:
			if(_ahb_presc > 0)
				freq /= _ahb_presc;
			break;
		
		case RCC_SRC_APB1:
		case RCC_SRC_APB2:
			if(_ahb_presc > 0)
				freq /= _ahb_presc;
			if(_apb_presc > 0)
				freq /= _apb_presc;
			break;
	}
	
	return freq;
}

void hal::rcc_reset(void)
{
	NVIC_SystemReset();
}

rcc_rst_reason_t hal::rcc_get_rst_reason(void)
{
	if(!csr)
		rcc_init();
	
	if(csr & RCC_CSR_PINRSTF)
		return RCC_RST_REASON_EXTERNAL;
	else if(csr & RCC_CSR_PORRSTF)
		return RCC_RST_REASON_LOW_POWER;
	else if(csr & RCC_CSR_SFTRSTF)
		return RCC_RST_REASON_INTERNAL;
	else if(csr & RCC_CSR_IWDGRSTF)
		return RCC_RST_REASON_WDT;
	else if(csr & RCC_CSR_WWDGRSTF)
		return RCC_RST_REASON_WDT;
	else if (csr & RCC_CSR_LPWRRSTF)
		return RCC_RST_REASON_LOW_POWER;
	return RCC_RST_REASON_UNKNOWN;
}