#include <stddef.h>
#include <string.h>
#include "common/assert.h"
#include "rtc.hpp"
#include "systick/systick.hpp"
#include "CMSIS/Device/STM32F4xx/Include/stm32f4xx.h"

using namespace hal;

constexpr auto init_timeout = 10; // ms

static void *_ctx = NULL;
static rtc::cb_t _cb = NULL;

static int8_t first_setup(void);
static void config_lsi(void);
static int8_t config_lse(void);
static int8_t enter_init(void);

int8_t rtc::init(clk_t clk)
{
	// Backup domain software force reset
	//RCC->BDCR |= RCC_BDCR_BDRST;
	//RCC->BDCR &= ~RCC_BDCR_BDRST;
	
	// PWR periphery clock enable
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	
	// Disable backup domain write protection
	PWR->CR |= PWR_CR_DBP;
	
	if(clk == CLK_LSI)
		config_lsi();
	else
	{
		if(config_lse())
			return -1;
	}
	
	// Enable RTC clk
	RCC->BDCR |= RCC_BDCR_RTCEN;
	
	// If rtc isn't running yet (if year field is equal to 0)
	if(!(RTC->ISR & RTC_ISR_INITS) && !first_setup())
		return -1;
	
	NVIC_ClearPendingIRQ(RTC_Alarm_IRQn);
	NVIC_SetPriority(RTC_Alarm_IRQn, 7);
	NVIC_EnableIRQ(RTC_Alarm_IRQn);
	
	return 0;
}

struct tm rtc::get()
{
	struct tm time = {};
	
	uint32_t tmp_time = RTC->TR;
	uint32_t tmp_date = RTC->DR;
	
	time.tm_year = ((tmp_date & RTC_DR_YT_Msk) >> RTC_DR_YT_Pos) * 10;
	time.tm_year += (tmp_date & RTC_DR_YU_Msk) >> RTC_DR_YU_Pos;
	
	time.tm_wday = (tmp_date & RTC_DR_WDU_Msk) >> RTC_DR_WDU_Pos;
	
	time.tm_mon = ((tmp_date & RTC_DR_MT_Msk) >> RTC_DR_MT_Pos) * 10;
	time.tm_mon += (tmp_date & RTC_DR_MU_Msk) >> RTC_DR_MU_Pos;
	
	time.tm_mday = ((tmp_date & RTC_DR_DT_Msk) >> RTC_DR_DT_Pos) * 10;
	time.tm_mday += (tmp_date & RTC_DR_DU_Msk) >> RTC_DR_DU_Pos;
	
	time.tm_hour = ((tmp_time & RTC_TR_HT_Msk) >> RTC_TR_HT_Pos) * 10;
	time.tm_hour += (tmp_time & RTC_TR_HU_Msk) >> RTC_TR_HU_Pos;
	
	time.tm_min = ((tmp_time & RTC_TR_MNT_Msk) >> RTC_TR_MNT_Pos) * 10;
	time.tm_min += (tmp_time & RTC_TR_MNU_Msk) >> RTC_TR_MNU_Pos;
	
	time.tm_sec = ((tmp_time & RTC_TR_ST_Msk) >> RTC_TR_ST_Pos) * 10;
	time.tm_sec += (tmp_time & RTC_TR_SU_Msk) >> RTC_TR_SU_Pos;
	
	// Normalize time to comply struct tm format
	time.tm_mon--;
	time.tm_wday--;
	time.tm_year += 100;
	
	return time;
}

int8_t rtc::set(struct tm &time)
{
	ASSERT(is_valid(time));
	
	// Normalize time for STM32 RTC periphery
	time.tm_mon++;
	time.tm_wday++;
	time.tm_year %= 100;
		
	// Disable write protection
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;
	
	uint32_t tmp_date = (time.tm_year / 10) << RTC_DR_YT_Pos;
	tmp_date |= (time.tm_year % 10) << RTC_DR_YU_Pos;
	tmp_date |= time.tm_wday << RTC_DR_WDU_Pos;
	tmp_date |= (time.tm_mon / 10) << RTC_DR_MT_Pos;
	tmp_date |= (time.tm_mon % 10) << RTC_DR_MU_Pos;
	tmp_date |= (time.tm_mday / 10) << RTC_DR_DT_Pos;
	tmp_date |= (time.tm_mday % 10) << RTC_DR_DU_Pos;
	
	uint32_t tmp_time = (time.tm_hour / 10) << RTC_TR_HT_Pos;
	tmp_time |= (time.tm_hour % 10) << RTC_TR_HU_Pos;
	tmp_time |= (time.tm_min / 10) << RTC_TR_MNT_Pos;
	tmp_time |= (time.tm_min % 10) << RTC_TR_MNU_Pos;
	tmp_time |= (time.tm_sec / 10) << RTC_TR_ST_Pos;
	tmp_time |= (time.tm_sec % 10) << RTC_TR_SU_Pos;
	
	int8_t res = enter_init();
	if(res)
		goto Exit;
	
	RTC->TR = tmp_time;
	RTC->DR = tmp_date;
	// Exit initialization mode
	RTC->ISR &= ~RTC_ISR_INIT;
	
Exit:
	// Write protection enable
	RTC->WPR = 0xFF;
	return res;
}

void rtc::bckp_write(uint8_t addr, void *buff, size_t size)
{
	
}

void rtc::bckp_read(uint8_t addr, void *buff, size_t size)
{
	
}

bool rtc::is_valid(struct tm &time)
{
	return time.tm_sec <= 59 && time.tm_min <= 59 && time.tm_hour <= 23 &&
		time.tm_mday >= 1 && time.tm_mday <= 31 && time.tm_mon <= 11 &&
		/* tm_year should be relative to 1900 year.
		Suppose it is at least 2000 year now */
		time.tm_year > 100 &&
		time.tm_wday <= 6 && time.tm_yday <= 365;
}

void rtc::set_alarm_cb(cb_t cb, void *ctx)
{
	_cb = cb;
	_ctx = ctx;
}

void rtc::set_alarm(struct tm time)
{
	// Disable write protection
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;
	
	// Disable alarm A and its interrupt
	RTC->CR &= ~(RTC_CR_ALRAE_Msk | RTC_CR_ALRAIE_Msk);
	
	struct tm time_all_0 = {};
	if(!memcmp(&time, &time_all_0, sizeof(time)))
	{
		// Enable write protection
		RTC->WPR = 0xFF;
		EXTI->IMR &= ~EXTI_IMR_MR17;
		EXTI->RTSR &= ~EXTI_IMR_MR17;
		return;
	}
	
	// Check that the RTC->ALRMAR register can be accessed
	while(!(RTC->ISR & RTC_ISR_ALRAWF_Msk));
	
	// Alarm every second
	RTC->ALRMAR |= RTC_ALRMAR_MSK4 | RTC_ALRMAR_MSK3 | RTC_ALRMAR_MSK2 |
		RTC_ALRMAR_MSK1;
	
	// Enable alarm A and its interrupt
	RTC->CR |= RTC_CR_ALRAE_Msk | RTC_CR_ALRAIE_Msk;
	
	// Enable write protection
	RTC->WPR = 0xFF;
	
	EXTI->IMR |= EXTI_IMR_MR17;
	EXTI->RTSR |= EXTI_IMR_MR17;
}

static int8_t first_setup(void)
{
	// Disable write protection
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;
	
	if(enter_init())
		return -1;
	
	// 24 hour selection
	RTC->CR |= RTC_CR_FMT;
	
	// Bypass the shadow registers enable
	RTC->CR |= RTC_CR_BYPSHAD;
	
	/* Prescaler setup for 32768 Hz (1 Hz output):
	(32768 / PREDIV_ASYNC) / PREDIV_SYNC = 1
	PREDIV_ASYNC = 128, PREDIV_SYNC = 256 */
	uint16_t sync_presc = 256 - 1;
	uint16_t async_presc = 128 - 1;
	
	RTC->PRER |= sync_presc | (async_presc << 16);
	
	// Exit from initialization mode
	RTC->ISR &= ~RTC_ISR_INIT;
	
	// Enable write protection
	RTC->WPR = 0xFF;
	
	return 0;
}

static void config_lsi(void)
{
	RCC->CSR |= RCC_CSR_LSION;
	while(!(RCC->CSR & RCC_CSR_LSIRDY));
	
	// Setup RTC clock source
	RCC->BDCR &= ~RCC_BDCR_RTCSEL_0;
	RCC->BDCR |= RCC_BDCR_RTCSEL_1;
}

static int8_t config_lse(void)
{
	RCC->BDCR |= RCC_BDCR_LSEON;
	uint32_t last_tim = systick::get_ms();
	while(1)
	{
		if(RCC->BDCR & RCC_BDCR_LSERDY)
		{
			// Setup RTC clock source
			RCC->BDCR &= ~RCC_BDCR_RTCSEL;
			RCC->BDCR |= RCC_BDCR_RTCSEL_0;
			return 0;
		}
		else if(systick::get_past(last_tim) >= init_timeout)
			return -1;
	}
}

static int8_t enter_init(void)
{
	// Request initialization mode
	RTC->ISR = 0xFFFFFFFF;
	uint32_t last_tim = systick::get_ms();
	while(1)
	{
		if(RTC->ISR & RTC_ISR_INITF)
			return 0;
		
		if(systick::get_past(last_tim) >= init_timeout)
			return -1;
	}
}

extern "C" void rtc_irq_hndlr()
{
	if(_cb)
		_cb(rtc::get(), _ctx);
}

extern "C" void RTC_Alarm_IRQHandler(void)
{
	rtc_irq_hndlr();
}