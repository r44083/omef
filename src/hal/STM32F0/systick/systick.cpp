#include "systick.hpp"
#include "rcc/rcc.hpp"
#include "CMSIS/Device/STM32F0xx/Include/stm32f0xx.h"
#include "CMSIS/Include/core_cm0.h"
#include "FreeRTOS.h"
#include "task.h"

using namespace hal;

void systick::init(void)
{
	uint32_t systick_freq = rcc_get_freq(RCC_SRC_AHB);
	SysTick_Config(systick_freq / configTICK_RATE_HZ);
}

uint32_t systick::get_ms(void)
{
	return xTaskGetTickCount();
}

uint32_t systick::get_past(uint32_t start)
{
	return xTaskGetTickCount() - start;
}

/* Implemented in third_party/FreeRTOS/port.c
extern "C" void SysTick_Handler(void)
{
	systick_cnt++;
}
*/