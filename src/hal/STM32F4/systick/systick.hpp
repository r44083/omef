#pragma once

#include <stdint.h>

namespace hal
{
/**
 * @brief      Initialization of systick timer
 */
void systick_init(void);

/**
 * @brief      Get systick timer value
 *
 * @return     systick timer value in ms
 */
uint32_t systick_get_ms(void);

/**
 * @brief      Get the difference between current systick value and start value
 *
 * @param[in]  start  systick value since what need to take difference
 *
 * @return     Difference between current systick value and start value
 */
uint32_t systick_get_past(uint32_t start);
}