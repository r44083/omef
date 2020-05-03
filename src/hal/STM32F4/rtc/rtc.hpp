#pragma once

#include <stdint.h>
#include <stddef.h>
#include <time.h>

namespace hal { class rtc; }
// For internal use only! (called from ISR)
extern "C" void rtc_irq_hndlr();

namespace hal
{
class rtc
{
	public:
		enum clk_t
		{
			CLK_LSI,
			CLK_LSE
		};
		
		/**
		 * @brief      RTC initialization
		 *
		 * @param[in]  clk   Clock source for RTC
		 *
		 * @return     0 in case of success, otherwise - negative value
		 */
		static int8_t init(clk_t clk);
		
		/**
		 * @brief         Get RTC time
		 *
		 * @param[in,out] time  Pointer to the time_t struct
		 */
		static struct tm get();
		
		/**
		 * @brief      Set RTC time
		 *
		 * @param[in]  time  Pointer to the time_t struct
		 *
		 * @return     0 in case of success, otherwise - negative value
		 */
		static int8_t set(struct tm &time);
		
		/**
		 * @brief      Write the array into the backup registers
		 *
		 * @param[in]  addr  Offset in backup registers
		 * @param[in]  buff  Pointer to the buffer
		 * @param[in]  size  Size of the buffer, which should be written
		 */
		static void bckp_write(uint8_t addr, void *buff, size_t size);
		
		/**
		 * @brief         Read array from backup registers
		 *
		 * @param[in]     addr  Offset in backup registers
		 * @param[in,out] buff  Pointer to the buffer
		 * @param[in]     size  Size of the buffer, which should be filled
		 */
		static void bckp_read(uint8_t addr, void *buff, size_t size);
		
		typedef void (*cb_t)(struct tm time, void *ctx);
		
		static void set_alarm_cb(cb_t cb, void *ctx);
		static void set_alarm(struct tm time);
		
		static bool is_valid(struct tm &time);
	private:
		rtc() {};
		friend void ::rtc_irq_hndlr();
};
};