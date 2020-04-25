#pragma once

#include <stdint.h>

#include "gpio/gpio.hpp"
#include "spi/spi.hpp"
#include "FreeRTOS.h"
#include "semphr.h"

namespace drv
{
class dataflash
{
	public:
		enum res_t
		{
			RES_OK =  0,
			RES_SPI_ERR = -1,
			RES_ERASE_PORGRAM_ERR = -2,
			RES_UNSUPPORTED_CHIP_ERR = -3,
			RES_NO_RESPONSE_ERR = -4
		};
		
		enum page_size_t
		{
			/* Standard DataFlash page size
			264, 528, 1056 or 2112 bytes */
			PAGE_SIZE_DATAFLASH,
			
			/* "power of 2" (binary) binary page size
			256, 512, 1024 or 2048 bytes */
			PAGE_SIZE_POW_2
		};
		
		dataflash(hal::spi &spi, hal::gpio &cs, hal::gpio *wp = NULL,
			hal::gpio *rst = NULL);
		~dataflash();
		
		int8_t init(page_size_t page_size = PAGE_SIZE_DATAFLASH);
		
		struct info_t
		{
			uint16_t page_size;
			uint16_t pages;
		};
		info_t info() { return _info; }
		
		int8_t read(void *buff, uint16_t page, uint16_t pages = 1);
		int8_t write(void *buff, uint16_t page, uint16_t pages = 1);
		int8_t erase(uint16_t page, uint16_t pages = 1);
		
		enum density_t
		{
			DENSITY_1_MBIT   = 2,
			DENSITY_2_MBIT   = 3,
			DENSITY_4_MBIT   = 4,
			DENSITY_8_MBIT   = 5,
			DENSITY_16_MBIT  = 6,
			DENSITY_32_MBIT  = 7,
			DENSITY_64_MBIT  = 8,
			DENSITY_128_MBIT = 9,
			DENSITY_256_MBIT = 10,
			DENSITY_512_MBIT = 11
		};
		enum family_t
		{
			FAMILY_AT45DXXX        = 1,
			FAMILY_AT26DFXXX       = 2,
			FAMILY_AT25F_AT25FSXXX = 3
		};
#pragma pack(push, 1)
		/* Based on JEDEC publication 106 (JEP106), Manufacturer ID data can be
		comprised of any number of bytes. Some manufacturers may have
		Manufacturer ID codes that are two, three or even four bytes long with
		the first byte(s) in the sequence being 7FH. A system should detect code
		7FH as a "Continuation Code" and continue to read Manufacturer ID bytes.
		The first non-7FH byte would signify the last byte of Manufacturer ID
		data. For Atmel (and some other manufacturers), the Manufacturer ID data
		is comprised of only one byte. */
		//uint8_t jedec_continuation_number;
		
		struct jedec_t
		{
			uint8_t manufacturer_id:8; // JEDEC Assigned Code
			struct
			{
				density_t density_code:5;
				family_t family_code:3;
				uint8_t product_vers_code:5;
				uint8_t sub_code:3;
			} dev_id;
			/* Indicate number of additional bytes with extended info. As
			indicated in the JEDEC Standard, reading the EDI String Length and
			any subsequent data is optional. */
			uint8_t extended_dev_info_len:8;
		};
#pragma pack(pop)
		
		int8_t jedec(jedec_t &jedec);
	
	private:
		/* 0   bit allways equal to 0
		   3:1 bits are significant
		   
		   density in Mbits for <= 64 MBit dataflash:
		   2^(significant_bits - 1) */
		enum density_code_t
		{
			DENSITY_CODE_1_MBIT   = (1 << 1) | 0b0001,
			DENSITY_CODE_2_MBIT   = (2 << 1) | 0b0001,
			DENSITY_CODE_4_MBIT   = (3 << 1) | 0b0001,
			DENSITY_CODE_8_MBIT   = (4 << 1) | 0b0001,
			DENSITY_CODE_16_MBIT  = (5 << 1) | 0b0001,
			DENSITY_CODE_32_MBIT  = (6 << 1) | 0b0001,
			DENSITY_CODE_64_MBIT  = (7 << 1) | 0b0001,
			DENSITY_CODE_128_MBIT = 4,
			DENSITY_CODE_256_MBIT = 6,
			DENSITY_CODE_512_MBIT = 8
		};
		
#pragma pack(push, 1)
		struct status_t
		{
			// ----------- 1-st byte of status register
			page_size_t page_size:1;
			bool sector_protection:1;
			density_code_t density_code:4;
			
			/* 0 - Main memory page data matches buffer data
			   1 - Main memory page data does not match buffer data */
			bool compare_result:1;
			bool ready:1;
			
			// ----------- 2-nd byte of status register (optional)
			/* 0 - No sectors are erase suspended
			   1 - A sector is erase suspended */
			bool erase_suspend:1;
			
			/* 0 - No program operation has been suspended while using Buffer 1
			   1 - A sector is program suspended while using Buffer 1
			*/
			bool program_suspend_status_buff_1:1;
			
			/* 0 - No program operation has been suspended while using Buffer 2
			   1 - A sector is program suspended while using Buffer 2 */
			bool program_suspend_status_buff_2:1;
			
			/* 0 - Sector Lockdown command is disabled
			   1 - Sector Lockdown command is enabled */
			bool sector_lockdown:1;
			
			bool reserved2:1;
			bool erase_program_error:1;
			bool reserved1:1;
			
			/* 0 - Device is busy with an internal operation
			   1 - Device is ready */
			bool ready2:1;
		};
#pragma pack(pop)
		struct args
		{
			args() {};
			args(uint16_t page, uint16_t byte, info_t info);
			args(uint8_t arg1, uint8_t arg2, uint8_t arg3);
			
			uint8_t arg[4];
			
			// May equal to 4 if chip size >= 128 MBit, otherwise equal to 3
			uint8_t size;
		};
		
		hal::spi &_spi;
		hal::gpio &_cs;
		hal::gpio *_wp;
		hal::gpio *_rst;
		SemaphoreHandle_t api_lock;
		info_t _info;
		
		int8_t read_status(status_t &status);
		bool is_status_valid(const status_t &status);
		bool is_jedec_valid(const jedec_t &jedec);
		info_t get_info(density_code_t density_code, page_size_t page_size);
		density_code_t get_density_code(info_t &info);
		
		enum class cmd : uint8_t
		{
			// Read commands
			CONTINUOUS_ARRAY_READ                                    = 0xE8, // Legacy command
			CONTINUOUS_ARRAY_READ_HI_FREQ                            = 0x0B,
			CONTINUOUS_ARRAY_READ_LOW_FREQ                           = 0x03,
			
			MAIN_MEMORY_PAGE_READ                                    = 0xD2,
			BUFFER1_READ                                             = 0xD4,
			BUFFER2_READ                                             = 0xD6,
			STATUS_REG_READ                                          = 0xD7,
			
			SECURITY_REG_READ                                        = 0x77, // Present since "D" family
			SECURITY_REG_WRITE                                       = 0x9A, // Present since "D" family
			
			// Supported only by new AT45DBxxxD serial (unlike AT45DBxxxB)
			READ_MANUFACTURER_AND_DEVICE_ID                          = 0x9F,
			
			// Program and erase commands
			BUFFER1_WRITE                                            = 0x84,
			BUFFER2_WRITE                                            = 0x87,
			BUFFER1_TO_MAIN_MEMORY_PAGE_PROGRAM_WITH_BUILTIN_ERASE   = 0x83,
			BUFFER2_TO_MAIN_MEMORY_PAGE_PROGRAM_WITH_BUILTIN_ERASE   = 0x86,
			BUFFER1_TO_MAIN_MEMORY_PAGE_PROGRAM_WITHOT_BUILTIN_ERASE = 0x88,
			BUFFER2_TO_MAIN_MEMORY_PAGE_PROGRAM_WITHOT_BUILTIN_ERASE = 0x89,
			PAGE_ERASE                                               = 0x81,
			BLOCK_ERASE                                              = 0x50,
			MAIN_MEMORY_PAGE_PROGRAM_THROUGH_BUFFER1                 = 0x82,
			MAIN_MEMORY_PAGE_PROGRAM_THROUGH_BUFFER2                 = 0x85,
			SECTOR_ERASE                                             = 0x7C, // Present since "D" family
			
			// Additional commands
			MAIN_MEMORY_PAGE_TO_BUFFER1_TRANSFER                     = 0x53,
			MAIN_MEMORY_PAGE_TO_BUFFER2_TRANSFER                     = 0x55,
			MAIN_MEMORY_PAGE_TO_BUFFER1_COMPARE                      = 0x60,
			MAIN_MEMORY_PAGE_TO_BUFFER2_COMPARE                      = 0x61,
			AUTO_PAGE_REWRITE_THROUGH_BUFFER1                        = 0x58,
			AUTO_PAGE_REWRITE_THROUGH_BUFFER2                        = 0x59,
			
			ENTER_DEEP_POWER_DOWN                                    = 0xB9, // Present since "D" family
			EXIT_DEEP_POWER_DOWN                                     = 0xAB, // Present since "D" family
		};
		int8_t cmd(enum cmd cmd, args args, uint32_t timeout);
		int8_t cmd_with_write(enum cmd cmd, args args, void *buff, size_t size,
			uint32_t timeout);
		int8_t cmd_with_read(enum cmd cmd, args args, void *buff, size_t size,
			uint8_t dummy_bytes);
		
		int8_t wait_ready(uint32_t timeout_ms, status_t *status = NULL);
		int8_t set_page_size(page_size_t page_size);
		uint32_t get_timeout(enum cmd cmd, info_t &info);
		uint32_t get_chip_erase_timeout(info_t &info);
};
}
