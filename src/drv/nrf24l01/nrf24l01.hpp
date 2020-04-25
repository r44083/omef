#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "gpio/gpio.hpp"
#include "spi/spi.hpp"
#include "exti/exti.hpp"
#include "tim/tim.hpp"
#include "FreeRTOS.h"
#include "semphr.h"

namespace drv
{
#define NRF24L01_PLUS 1 // Support nrf24l01+ (instead of nrf24l01)

class nrf24l01
{
	public:
		enum res_t
		{
			RES_OK                  =  0,
			RES_SPI_ERR             = -1,
			RES_TX_MAX_RETRIES_ERR  = -2,
			RES_NRF_NO_RESPONSE_ERR = -3
		};
		
		static constexpr auto pipes = 6;
		static constexpr auto fifo_size = 32;
		
		nrf24l01(hal::spi &spi, hal::gpio &cs, hal::gpio &ce, hal::exti &exti,
			hal::tim &tim);
		~nrf24l01();
		
		int8_t init();
		
		/**
		 * @brief Configure pipe for reception.
		 * 
		 * @note Pipe 0 is used in tx setup to receive auto ack. So, don't
		 * use pipe 0 for reading if any tx api will be used.
		 * 
		 * @param pipe Index of pipe.
		 * @param addr 5-byte rx address.
		 * @param size Payload size: 1 to 32 bytes.
		 * @param auto_ack Is auto acknowledgment enabled.
		 * @return int8_t Error code. @see @ref res_t
		 */
		int8_t open_pipe(uint8_t pipe, uint64_t addr, uint8_t size = fifo_size,
			bool auto_ack = true);
		
		struct rx_packet_t
		{
			uint8_t pipe;
			uint8_t buff[fifo_size];
			size_t size;
		};
		/**
		 * @brief Read packet from pipe. @ref open_pipe() should be executed
		 * before to initialize pipe.
		 * 
		 * @note This method changes nrf24l01 mode from power down to rx.
		 * 
		 * @param packet Packet to receive.
		 * @param tx_ack Ack payload, which will be transmitted after reception.
		 * @return int8_t Error code. @see @ref res_t
		 */
		int8_t read(rx_packet_t &packet, uint8_t tx_ack[fifo_size] = NULL);
		
		/**
		 * @brief Close pipe. If it was last opened pipe, power down mode will
		 * be activated.
		 * 
		 * @param pipe Index of pipe.
		 * @return int8_t Error code. @see @ref res_t
		 */
		int8_t close_pipe(uint8_t pipe);
		
		/**
		 * @brief Set tx address and configure rx pipe 0 to receive auto ack.
		 * 
		 * @note Pipe 0 will be also reconfigured for auto ack.
		 * 
		 * @param addr 5-byte tx address. It is also set to pipe 0 rx address.
		 * @return int8_t Error code. @see @ref res_t
		 */
		int8_t tx_addr(uint64_t addr);
		
		/**
		 * @brief Write data buffer.
		 * 
		 * @note This method changes nrf24l01 mode from power down to tx.
		 * 
		 * @param buff Pointer to data buffer.
		 * @param size Size of data buffer. It must match the payload size on
		 * receiving side.
		 * @param rx_ack Ack payload, which was received after transmitting.
		 * @param is_continuous_tx Is further continuous transmission planned.
		 * In case of continuous writing, tx mode will be enabled all the time
		 * to avoid redundant 130 us (standby-1 mode -> tx mode) delay.
		 * @return int8_t Error code. @see @ref res_t
		 */
		int8_t write(void *buff, size_t size, void *rx_ack = NULL,
			bool is_continuous_tx = false);
		
		/**
		 * @brief Go to power down mode.
		 * Usually this method shouldn't be used in common workflow. Power down
		 * mode is managed automatically by this driver.
		 * It might by used after write() method when no more future tx/rx
		 * actions is planned.
		 * Power down mode is automatically activated when none of the rx pipes
		 * are opened.
		 * 
		 * @return int8_t Error code. @see @ref res_t
		 */
		int8_t power_down();
		
	private:
		enum class reg : uint8_t
		{
			CONFIG,
			EN_AA,
			EN_RXADDR,
			SETUP_AW,
			SETUP_RETR,
			RF_CH,
			RF_SETUP,
			STATUS,
			OBSERVE_TX,
			RPD, // Named "CD (Carrier Detect)" in nrf24l01 documentation
			RX_ADDR_P0,
			RX_ADDR_P1,
			RX_ADDR_P2,
			RX_ADDR_P3,
			RX_ADDR_P4,
			RX_ADDR_P5,
			TX_ADDR,
			RX_PW_P0,
			RX_PW_P1,
			RX_PW_P2,
			RX_PW_P3,
			RX_PW_P4,
			RX_PW_P5,
			FIFO_STATUS,
		#if NRF24L01_PLUS
			DYNPD = 28,
			FEATURE = 29
		#endif
		};
		int8_t read_reg(reg reg, void *data, size_t size = 1);
		int8_t write_reg(reg reg, void *data, size_t size = 1);
		
		enum class instruction : uint8_t
		{
			R_REGISTER         = 0x00,
			W_REGISTER         = 0x20,
			R_RX_PAYLOAD       = 0x61,
			W_TX_PAYLOAD       = 0xA0,
			FLUSH_TX           = 0xE1,
			FLUSH_RX           = 0xE2,
			REUSE_TX_PL        = 0xE3,
		#if NRF24L01_PLUS
			ACTIVATE           = 0x50,
			R_RX_PL_WID        = 0x60,
			W_ACK_PAYLOAD      = 0xA8,
			W_TX_PAYLOAD_NOACK = 0xB0,
		#endif
			NOP                = 0xFF
		};
		int8_t exec_instruction(instruction instruction);
		int8_t exec_instruction_with_read(instruction instruction, void *buff,
			size_t size);
		int8_t exec_instruction_with_write(instruction instruction, void *buff,
			size_t size);
		
		enum class mode { PWR_DOWN, STANDBY_1, TX, RX };
		int8_t set_mode(mode new_mode);
		
		void delay(uint32_t us);
		
		static void exti_cb(hal::exti *exti, void *ctx);
		static void tim_cb(hal::tim *tim, void *ctx);
		
		hal::spi &_spi;
		hal::gpio &_cs;
		hal::gpio &_ce;
		hal::exti &_exti;
		hal::tim &_tim;
		SemaphoreHandle_t api_lock;
		
		struct
		{
			mode _mode;
			struct
			{
				bool is_open;
				uint8_t size;
			} pipe[pipes];
		} conf;
};
}
