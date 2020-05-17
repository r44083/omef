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
		
		enum class dev {NRF24L01, NRF24L01_PLUS};
		
		static constexpr auto pipes = 6;
		static constexpr auto fifo_size = 32;
		
		nrf24l01(hal::spi &spi, hal::gpio &cs, hal::gpio &ce, hal::exti &exti,
			hal::tim &tim, dev dev = dev::NRF24L01_PLUS);
		~nrf24l01();
		
		int8_t init();
		
#pragma pack(push, 1) // Reduce size of conf_t struct
		struct conf_t
		{
			struct
			{
				bool enable:1;
				uint8_t size:6;
				uint64_t addr:40;
				bool auto_ack:1;
				bool dyn_payload:1;
			} pipe[pipes];
			uint64_t tx_addr:40;
			bool tx_auto_ack:1;
			bool dyn_payload:1;
		};
#pragma pack(pop)
		int8_t get_conf(conf_t &conf);
		int8_t set_conf(conf_t &conf);
		
		struct rx_packet_t
		{
			uint8_t pipe;
			uint8_t buff[fifo_size];
			uint8_t size;
		};
		struct ack_payload_t
		{
			uint8_t buff[fifo_size];
			uint8_t size;
		};
		/**
		 * @brief Read packet from pipe. @ref open_pipe() should be executed
		 * before to initialize pipe.
		 * 
		 * @note This method changes nrf24l01 mode from power down to rx.
		 * 
		 * @param packet Packet to receive.
		 * @param ack Ack payload, which will be transmitted after reception.
		 * @return int8_t Error code. @see @ref res_t
		 */
		int8_t read(rx_packet_t &packet, ack_payload_t *ack = NULL);
		
		/**
		 * @brief Write data buffer.
		 * 
		 * @note This method changes nrf24l01 mode from power down to tx.
		 * 
		 * @param buff Pointer to data buffer.
		 * @param size Size of data buffer. It must match the payload size on
		 * receiving side.
		 * @param ack Ack payload, which was received after transmitting.
		 * @param is_continuous_tx Is further continuous transmission planned.
		 * In case of continuous writing, tx mode will be enabled all the time
		 * to avoid redundant 130 us (standby-1 mode -> tx mode) delay.
		 * @return int8_t Error code. @see @ref res_t
		 */
		int8_t write(void *buff, size_t size, ack_payload_t *ack = NULL,
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
			DYNPD = 28, // Only for nrf24l01+
			FEATURE = 29 // Only for nrf24l01+
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
			ACTIVATE           = 0x50, // Only for nrf24l01+
			R_RX_PL_WID        = 0x60, // Only for nrf24l01+
			W_ACK_PAYLOAD      = 0xA8, // Only for nrf24l01+
			W_TX_PAYLOAD_NOACK = 0xB0, // Only for nrf24l01+
			NOP                = 0xFF
		};
		int8_t exec_instruction(instruction instruction);
		int8_t exec_instruction_with_read(instruction instruction, void *buff,
			size_t size);
		int8_t exec_instruction_with_write(instruction instruction, void *buff,
			size_t size);
		
		enum class mode { PWR_DOWN, STANDBY_1, TX, RX };
		int8_t set_mode(mode mode);
		
		bool is_conf_valid(conf_t &conf);
		int8_t ack_payload(bool enable);
		
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
			enum dev dev;
			enum mode mode;
			struct
			{
				uint8_t size;
				bool dyn_payload;
			} pipe[pipes];
			bool ack_payload;
			bool dyn_payload;
		} _conf;
};
}