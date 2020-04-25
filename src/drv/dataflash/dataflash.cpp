#include <string.h>
#include "common/assert.h"
#include "common/freertos_wrappers.hpp"
#include "dataflash.hpp"
#include "FreeRTOS.h"
#include "task.h"

using namespace drv;

constexpr auto dataflash_family_code = 1;
constexpr auto jedec_continuation_code = 0x7F;

// Present since "D" family
constexpr uint8_t cmd_set_page_size_pow2[] = {0x3D, 0x2A, 0x80, 0xA6};
constexpr uint8_t cmd_set_page_size_dataflash[] = {0x3D, 0x2A, 0x80, 0xA7};
constexpr uint8_t cmd_enable_sector_protection[] = {0x3D, 0x2A, 0x7F, 0xA9};
constexpr uint8_t cmd_disable_sector_protection[] = {0x3D, 0x2A, 0x7F, 0x9A};
constexpr uint8_t cmd_enable_sector_lockdown[] = {0x3D, 0x2A, 0x7F, 0x30};
constexpr uint8_t cmd_chip_erase[] = {0xC7, 0x94, 0x80, 0x9A};

dataflash::dataflash(hal::spi &spi, hal::gpio &cs, hal::gpio *wp,
	hal::gpio *rst):
	_spi(spi),
	_cs(cs),
	_wp(wp),
	_rst(rst),
	_info({})
{
	ASSERT(_spi.bit_order() == hal::spi::BIT_ORDER_MSB);
	ASSERT(_cs.mode() == hal::gpio::MODE_DO);
	ASSERT(!_wp || _wp->mode() == hal::gpio::MODE_DO);
	ASSERT(!_rst || _rst->mode() == hal::gpio::MODE_DO);
	
	_cs.set(1);
	ASSERT((api_lock = xSemaphoreCreateMutex()));
}

dataflash::~dataflash()
{
	_cs.set(1);
	xSemaphoreGive(api_lock);
	vSemaphoreDelete(api_lock);
}

int8_t dataflash::init(page_size_t page_size)
{
	int8_t res = set_page_size(page_size);
	if(res != RES_OK)
		return res;
	
	status_t status;
	if((res = read_status(status)))
		return res;
	
	_info = get_info(status.density_code, status.page_size);
	
	return res;
}

int8_t dataflash::read(void *buff, uint16_t page, uint16_t pages)
{
	ASSERT(buff);
	ASSERT(page + pages < _info.pages); // Not enough memory
	ASSERT(pages > 0);
	
	freertos::semaphore_take(api_lock, portMAX_DELAY);
	
	args args(page, 0, _info);
	
	int8_t res = cmd_with_read(cmd::CONTINUOUS_ARRAY_READ_HI_FREQ, args, buff,
		_info.page_size * pages, 1);
	
	return res;
}

int8_t dataflash::write(void *buff, uint16_t page, uint16_t pages)
{
	ASSERT(buff);
	ASSERT(page + pages < _info.pages); // Not enough memory
	ASSERT(pages > 0);
	
	freertos::semaphore_take(api_lock, portMAX_DELAY);
	
	int res = RES_OK;
	enum cmd cmd = cmd::MAIN_MEMORY_PAGE_PROGRAM_THROUGH_BUFFER1;
	
	for(; pages; pages--, page++, buff = (uint8_t *)buff + _info.page_size)
	{
		args args(page, 0, _info);
		
		if((res = cmd_with_write(cmd, args, buff, _info.page_size,
			get_timeout(cmd, _info))))
		{
			break;
		}
	}
	
	return res;
}

int8_t dataflash::erase(uint16_t page, uint16_t pages)
{
	ASSERT(page + pages < _info.pages); // Not enough memory
	ASSERT(pages > 0);
	
	freertos::semaphore_take(api_lock, portMAX_DELAY);
	
	int8_t res = RES_OK;
	
	if (page == 0 && pages == _info.pages)
	{
		args args(cmd_chip_erase[1], cmd_chip_erase[2], cmd_chip_erase[3]);
		
		res = cmd(static_cast<enum cmd>(cmd_chip_erase[0]), args,
			get_timeout(static_cast<enum cmd>(cmd_chip_erase[0]), _info));
	}
	else
	{
		for(; pages; pages--, page++)
		{
			if((res = cmd(cmd::PAGE_ERASE, args(page, 0, _info),
				get_timeout(cmd::PAGE_ERASE, _info))))
			{
				break;
			}
		}
	}
	
	return res;
}

int8_t dataflash::jedec(jedec_t &jedec)
{
	freertos::semaphore_take(api_lock, portMAX_DELAY);
	hal::spi_cs cs(_cs);
	
	int8_t res;
	if((res = _spi.write(
		static_cast<uint8_t>(cmd::READ_MANUFACTURER_AND_DEVICE_ID))))
	{
		return RES_SPI_ERR;
	}
	
	uint8_t continuation_number;
	for (continuation_number = 0; continuation_number; continuation_number++)
	{
		// Read first byte of JEDEC (manufacturer_id)
		if((res = _spi.read(&jedec, sizeof(uint8_t))))
			return RES_SPI_ERR;
		
		// Skip continuation code
		if (jedec.manufacturer_id != jedec_continuation_code)
			break;
	}
	
	// Read rest part of JEDEC
	if((res = _spi.read(&jedec.dev_id, sizeof(jedec) - sizeof(uint8_t))))
		return RES_SPI_ERR;
	
	if(!is_jedec_valid(jedec))
		res = RES_UNSUPPORTED_CHIP_ERR;
	
	return res;
}

dataflash::args::args(uint16_t page, uint16_t byte, info_t info)
{
	uint8_t byte_addr_bits;
	
	switch(info.page_size)
	{
		case 256: byte_addr_bits = 9; break;
		case 264:
		case 512: byte_addr_bits = 10; break;
		case 528:
		case 1024: byte_addr_bits = 11; break;
		case 1056:
		case 2048: byte_addr_bits = 12; break;
		case 2112: byte_addr_bits = 13; break;
		default: ASSERT(0);
	}
	
	// If chip size >= 128 MBit, 4 bytes is required for addressing
	if(info.pages >= 16384)
	{
		arg[0] = page >> (24 - byte_addr_bits);
		arg[1] = page >> (16 - byte_addr_bits);
		arg[2] = page << (byte_addr_bits - 8) | byte >> 8;
		arg[3] = byte;
		size = 4;
	}
	else
	{
		arg[0] = page >> (16 - byte_addr_bits);
		arg[1] = page << (byte_addr_bits - 8) | byte >> 8;
		arg[2] = byte;
		size = 3;
	}
}

dataflash::args::args(uint8_t arg1, uint8_t arg2, uint8_t arg3)
{
	arg[0] = arg1;
	arg[1] = arg2;
	arg[2] = arg3;
	size = 3;
}

int8_t dataflash::read_status(status_t &status)
{
	hal::spi_cs cs(_cs);
	
	int8_t res;
	if((res = _spi.write(static_cast<uint8_t>(cmd::STATUS_REG_READ))))
		return RES_SPI_ERR;
	
	if((res = _spi.read(&status, sizeof(status))))
		return RES_SPI_ERR;
	
	if(!is_status_valid(status))
		res = RES_UNSUPPORTED_CHIP_ERR;
	
	return res;
}

bool dataflash::is_status_valid(const status_t &status)
{
	status_t all0x00 = {};
	status_t all0xFF;
	memset(&all0xFF, 0xFF, sizeof(all0xFF));
	
	if(!memcmp(&status, &all0x00, sizeof(status)) ||
		!memcmp(&status, &all0xFF, sizeof(status)))
	{
		return false;
	}
	
	switch(status.density_code)
	{
		case DENSITY_CODE_1_MBIT:
		case DENSITY_CODE_2_MBIT:
		case DENSITY_CODE_4_MBIT:
		case DENSITY_CODE_8_MBIT:
		case DENSITY_CODE_16_MBIT:
		case DENSITY_CODE_32_MBIT:
		case DENSITY_CODE_64_MBIT:
		case DENSITY_CODE_128_MBIT:
		case DENSITY_CODE_256_MBIT:
		case DENSITY_CODE_512_MBIT:
			break;
		
		default:
			return false;
	}
	
	return true;
}

bool dataflash::is_jedec_valid(const jedec_t &jedec)
{
	jedec_t all0x00 = {};
	jedec_t all0xFF;
	memset(&all0xFF, 0xFF, sizeof(all0xFF));
	
	if(!memcmp(&jedec, &all0x00, sizeof(jedec)) ||
		!memcmp(&jedec, &all0xFF, sizeof(jedec)))
	{
		return false;
	}
	
	if(jedec.dev_id.family_code != dataflash_family_code)
		return false;
	
	return true;
}

dataflash::info_t dataflash::get_info(density_code_t density_code,
	page_size_t page_size)
{
	info_t info = {};
	
	switch(density_code)
	{
		case DENSITY_CODE_1_MBIT:
			info.page_size = page_size == PAGE_SIZE_DATAFLASH ? 264 : 256;
			info.pages = 512;
			break;
		
		case DENSITY_CODE_2_MBIT:
			info.page_size = page_size == PAGE_SIZE_DATAFLASH ? 264 : 256;
			info.pages = 1024;
			break;
		
		case DENSITY_CODE_4_MBIT:
			info.page_size = page_size == PAGE_SIZE_DATAFLASH ? 264 : 256;
			info.pages = 2048;
			break;
		
		case DENSITY_CODE_8_MBIT:
			info.page_size = page_size == PAGE_SIZE_DATAFLASH ? 264 : 256;
			info.pages = 4096;
			break;
		
		case DENSITY_CODE_16_MBIT:
			info.page_size = page_size == PAGE_SIZE_DATAFLASH ? 528 : 512;
			info.pages = 4096;
			break;
		
		case DENSITY_CODE_32_MBIT:
			info.page_size = page_size == PAGE_SIZE_DATAFLASH ? 528 : 512;
			info.pages = 8192;
			break;
		
		case DENSITY_CODE_64_MBIT:
			info.page_size = page_size == PAGE_SIZE_DATAFLASH ? 1056 : 1024;
			info.pages = 8192;
			break;
		
		case DENSITY_CODE_128_MBIT:
			info.page_size = page_size == PAGE_SIZE_DATAFLASH ? 1056 : 1024;
			info.pages = 16384;
			break;
		
		case DENSITY_CODE_256_MBIT:
			info.page_size = page_size == PAGE_SIZE_DATAFLASH ? 2112 : 2048;
			info.pages = 16384;
			break;
		
		case DENSITY_CODE_512_MBIT:
			info.page_size = page_size == PAGE_SIZE_DATAFLASH ? 2112 : 2048;
			info.pages = 32768;
			break;
	}
	
	return info;
}

dataflash::density_code_t dataflash::get_density_code(info_t &info)
{
	switch(info.pages)
	{
		case 512:
			return DENSITY_CODE_1_MBIT;

		case 1024:
			return DENSITY_CODE_2_MBIT;

		case 2048:
			return DENSITY_CODE_4_MBIT;

		case 4096:
			if(info.page_size == 256 || info.page_size == 264)
				return DENSITY_CODE_8_MBIT;
			else
				return DENSITY_CODE_16_MBIT;

		case 8192:
			if(info.page_size == 512 || info.page_size == 528)
				return DENSITY_CODE_32_MBIT;
			else
				return DENSITY_CODE_64_MBIT;

		case 16384:
			if(info.page_size == 1024 || info.page_size == 1056)
				return DENSITY_CODE_128_MBIT;
			else
				return DENSITY_CODE_256_MBIT;

		case 32768:
			return DENSITY_CODE_512_MBIT;

		default:
			ASSERT(0);
			return (density_code_t)0;
	}
}

int8_t dataflash::cmd(enum cmd cmd, args args, uint32_t timeout)
{
	uint8_t cmd_buff[sizeof(cmd) + args.size];
	
	cmd_buff[0] = static_cast<uint8_t>(cmd);
	memcpy(&cmd_buff[1], &args.arg[0], args.size);
	
	int8_t res;
	if((res = _spi.write(cmd_buff, sizeof(cmd_buff), &_cs)))
		return RES_SPI_ERR;
	
	if(timeout)
		res = wait_ready(timeout);
	
	return res;
}

int8_t dataflash::cmd_with_write(enum cmd cmd, args args, void *buff,
	size_t size, uint32_t timeout)
{
	uint8_t cmd_buff[sizeof(cmd) + args.size];
	
	cmd_buff[0] = static_cast<uint8_t>(cmd);
	memcpy(&cmd_buff[1], &args.arg[0], args.size);
	
	_cs.set(0);
	
	int8_t res;
	if((res = _spi.write(cmd_buff, sizeof(cmd_buff))))
	{
		_cs.set(1);
		return RES_SPI_ERR;
	}
	
	if((res = _spi.write(buff, size)))
	{
		_cs.set(1);
		return RES_SPI_ERR;
	}
	
	_cs.set(1);
	
	if(timeout)
		res = wait_ready(timeout);
	
	return res;
}

int8_t dataflash::cmd_with_read(enum cmd cmd, args args, void *buff,
	size_t size, uint8_t dummy_bytes)
{
	uint8_t cmd_buff[sizeof(uint8_t) + args.size + dummy_bytes];
	
	cmd_buff[0] = static_cast<uint8_t>(cmd);
	memcpy(&cmd_buff[1], &args.arg[0], args.size);
	memset(&cmd_buff[1 + args.size], 0xFF, dummy_bytes);
	
	hal::spi_cs cs(_cs);
	
	int8_t res;
	if((res = _spi.write(cmd_buff, sizeof(cmd_buff))))
		return RES_SPI_ERR;
	
	if((res = _spi.read(buff, size)))
		return RES_SPI_ERR;
	
	return res;
}

int8_t dataflash::wait_ready(uint32_t timeout_ms, status_t *status)
{
	int8_t res;
	status_t _status;
	
	do
	{
		res = read_status(_status);
		if(res != RES_OK)
		{
			if(status)
				*status = _status;
			return res;
		}
		
		if(_status.erase_program_error)
		{
			res = RES_ERASE_PORGRAM_ERR;
			break;
		}
		
		if(_status.ready)
			break;
		
		vTaskDelay(1);
	} while(timeout_ms--);
	
	if(!timeout_ms)
		res = RES_NO_RESPONSE_ERR;
	
	if(status)
		*status = _status;
	
	return res;
}

int8_t dataflash::set_page_size(page_size_t page_size)
{
	const uint8_t *page_size_cmd = (page_size == PAGE_SIZE_DATAFLASH) ?
		cmd_set_page_size_dataflash : cmd_set_page_size_pow2;
	
	args args(page_size_cmd[1], page_size_cmd[2], page_size_cmd[3]);
	
	// Do not forget to update _info with new page size
	return cmd(static_cast<enum cmd>(page_size_cmd[0]), args, 0);
}

uint32_t dataflash::get_timeout(enum cmd cmd, info_t &info)
{
	if(cmd == static_cast<enum cmd>(cmd_chip_erase[0]))
		return get_chip_erase_timeout(info);
	
	static constexpr struct
	{
		enum cmd cmd;
		/* Timeout in ms:
		timeout[0] - 256/264 bytes in page
		timeout[1] - 512/526 bytes in page
		timeout[2] - 1024/1056 bytes in page
		timeout[3] - 2048/2112 bytes in page */
		uint16_t timeout[4];
	} timeouts[] =
	{
		{
			.cmd = cmd::MAIN_MEMORY_PAGE_PROGRAM_THROUGH_BUFFER1,
			.timeout = {35, 55, 55, 70}
		},
		{
			.cmd = cmd::PAGE_ERASE,
			.timeout = {25, 50, 45, 60}
		},
	};
	
	for(uint8_t i = 0; i < (sizeof(timeouts) / sizeof(timeouts[0])); i++)
	{
		if(cmd != timeouts[i].cmd)
			continue;

		switch(info.page_size)
		{
			case 256:
			case 264: return timeouts[i].timeout[0];
			case 512:
			case 526: return timeouts[i].timeout[1];
			case 1024:
			case 1056: return timeouts[i].timeout[2];
			case 2048:
			case 2112: return timeouts[i].timeout[3];
			default: ASSERT(0); return 0;
		}
	}
	
	ASSERT(0);
	return 0;
}

uint32_t dataflash::get_chip_erase_timeout(info_t &info)
{
	switch(info.pages)
	{
		case 512: return 2000;
		case 1024: return 4000;
		case 2048: return 17000;
		case 4096:
			if(info.page_size == 256 || info.page_size == 264)
				return 20000;
			else
				return 40000;
		case 8192:
			if(info.page_size == 512 || info.page_size == 528)
				return 80000;
			else
				return 208000;
		case 16384:
			if(info.page_size == 1024 || info.page_size == 1056)
				return 400000; // Approximate value
			else
				return 800000; // Approximate value
		case 32768: return 1600000; // Approximate value
		default: ASSERT(0); return 0;
	}
}