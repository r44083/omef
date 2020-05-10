#pragma once

#include <stdint.h>
#include <stdbool.h>

namespace drv::nrf24l01_priv
{
constexpr auto power_on_reset_timeout = 100; // ms
constexpr auto powerdown_to_standby1_timeout = 1500; // us
constexpr auto standby1_to_rxtx_timeout = 130; // us
constexpr auto transmit_max_timeout = 60; // ARD_4000_US * 15 retries = 60 ms
constexpr auto addr_max_size = 5;
constexpr auto addr_min_size = 1;

enum prim_rx_t
{
	PTX,
	PRX
};

enum crco_t
{
	_1_BYTE,
	_2_BYTE
};

enum aw_t
{
	_3_BYTES = 1,
	_4_BYTES = 2,
	_5_BYTES = 3
};

enum ard_t
{
	_250_US,
	_500_US,
	_750_US,
	_1000_US,
	_1250_US,
	_1500_US,
	_1750_US,
	_2000_US,
	_2250_US,
	_2500_US,
	_2750_US,
	_3000_US,
	_3250_US,
	_3500_US,
	_3750_US,
	_4000_US
};

enum rf_pwr_t
{
	_18_DBM,
	_12_DBM,
	_6_DBM,
	_0_DBM
};

#pragma pack(push, 1)
struct config_reg_t
{
	prim_rx_t prim_rx:1;
	bool pwr_up:1;
	crco_t crco:1;
	bool en_crc:1;
	bool mask_max_rt:1;
	bool mask_tx_ds:1;
	bool mask_rx_dr:1;
	bool reserved:1;
};

struct setup_aw_reg_t
{
	aw_t aw:2;
	uint8_t reserved:6;
};

struct setup_retr_reg_t
{
	uint8_t arc:4;
	ard_t ard:4;
};


union rf_setup_reg_t
{
	struct
	{
		bool lna_hcurr:1;
		rf_pwr_t rf_pwr:2;
		bool rf_dr:1;
		bool pll_lock:1;
		uint8_t reserved:3;
	} nrf24l01;
	struct
	{
		bool obselete:1;
		rf_pwr_t rf_pwr:2;
		bool rf_dr_high:1;
		bool pll_lock:1;
		bool rf_dr_low:1;
		bool reserved:1;
		bool cont_wave:1;
	} nrf24l01_plus;
};

struct status_reg_t
{
	bool tx_full:1;
	uint8_t rx_p_no:3;
	bool max_rt:1;
	bool tx_ds:1;
	bool rx_dr:1;
	bool reserved:1;
};

struct observe_tx_reg_t
{
	uint8_t arc_cnt:4;
	uint8_t plos_cnt:4;
};

struct fifo_status_reg_t
{
	bool rx_empty:1;
	bool rx_full:1;
	uint8_t reserved1:2;
	bool tx_empty:1;
	bool tx_full:1;
	bool tx_reuse:1;
	bool reserved2:1;
};

struct feature_reg_t // Only for nrf24l01+
{
	bool en_dyn_ack:1;
	bool en_ack_pay:1;
	bool en_dpl:1;
	uint8_t reserved:5;
};
#pragma pack(pop)
};
