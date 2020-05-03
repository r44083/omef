-include ../../../../config.mk

ESP8266_SRC := components/esp8266/source/chip_boot.c \
	components/esp8266/source/esp_err_to_name.c \
	components/esp8266/source/esp_timer.c \
	components/esp8266/source/esp_wifi_os_adapter.c \
	components/esp8266/source/esp_wifi.c \
	components/esp8266/source/ets_printf.c \
	components/esp8266/source/event_default_handlers.c \
	components/esp8266/source/event_loop.c \
	components/esp8266/source/phy_init.c \
	components/esp8266/source/reset_reason.c \
	components/esp8266/source/startup.c \
	components/esp8266/source/system_api.c \
	components/esp8266/source/task_wdt.c

FREERTOS_SRC := components/freertos/freertos/event_groups.c \
	components/freertos/freertos/list.c \
	components/freertos/freertos/queue.c \
	components/freertos/freertos/stream_buffer.c \
	components/freertos/freertos/tasks.c \
	components/freertos/freertos/timers.c \
	components/freertos/port/esp8266/freertos_hooks.c \
	components/freertos/port/esp8266/impure.c \
	components/freertos/port/esp8266/os_cpu_a.S \
	components/freertos/port/esp8266/panic.c \
	components/freertos/port/esp8266/port.c \
	components/freertos/port/esp8266/xtensa_context.S \
	components/freertos/port/esp8266/xtensa_vectors.S

HEAP_SRC := components/heap/port/esp8266/esp_heap_init.c \
	components/heap/src/esp_heap_caps.c \
	components/heap/src/esp_heap_trace.c

LOG_SRC := components/log/log.c

LWIP_SRC := components/lwip/apps/dhcpserver/dhcpserver.c \
	components/lwip/apps/multi-threads/sockets_mt.c \
	components/lwip/apps/ping/esp_ping.c \
	components/lwip/apps/ping/ping.c \
	components/lwip/lwip/src/api/api_lib.c \
	components/lwip/lwip/src/api/api_msg.c \
	components/lwip/lwip/src/api/err.c \
	components/lwip/lwip/src/api/netbuf.c \
	components/lwip/lwip/src/api/netdb.c \
	components/lwip/lwip/src/api/netifapi.c \
	components/lwip/lwip/src/api/sockets.c \
	components/lwip/lwip/src/api/tcpip.c \
	components/lwip/lwip/src/core/def.c \
	components/lwip/lwip/src/core/dns.c \
	components/lwip/lwip/src/core/inet_chksum.c \
	components/lwip/lwip/src/core/init.c \
	components/lwip/lwip/src/core/ip.c \
	components/lwip/lwip/src/core/mem.c \
	components/lwip/lwip/src/core/memp.c \
	components/lwip/lwip/src/core/netif.c \
	components/lwip/lwip/src/core/pbuf.c \
	components/lwip/lwip/src/core/raw.c \
	components/lwip/lwip/src/core/stats.c \
	components/lwip/lwip/src/core/sys.c \
	components/lwip/lwip/src/core/tcp_in.c \
	components/lwip/lwip/src/core/tcp_out.c \
	components/lwip/lwip/src/core/tcp.c \
	components/lwip/lwip/src/core/timeouts.c \
	components/lwip/lwip/src/core/udp.c \
	components/lwip/lwip/src/core/ipv4/autoip.c \
	components/lwip/lwip/src/core/ipv4/dhcp.c \
	components/lwip/lwip/src/core/ipv4/etharp.c \
	components/lwip/lwip/src/core/ipv4/icmp.c \
	components/lwip/lwip/src/core/ipv4/igmp.c \
	components/lwip/lwip/src/core/ipv4/ip4_addr.c \
	components/lwip/lwip/src/core/ipv4/ip4_frag.c \
	components/lwip/lwip/src/core/ipv4/ip4.c \
	components/lwip/lwip/src/core/ipv6/dhcp6.c \
	components/lwip/lwip/src/core/ipv6/ethip6.c \
	components/lwip/lwip/src/core/ipv6/icmp6.c \
	components/lwip/lwip/src/core/ipv6/inet6.c \
	components/lwip/lwip/src/core/ipv6/ip6_addr.c \
	components/lwip/lwip/src/core/ipv6/ip6_frag.c \
	components/lwip/lwip/src/core/ipv6/ip6.c \
	components/lwip/lwip/src/core/ipv6/mld6.c \
	components/lwip/lwip/src/core/ipv6/nd6.c \
	components/lwip/lwip/src/netif/ethernet.c \
	components/lwip/lwip/src/netif/ethernetif.c \
	components/lwip/port/esp8266/freertos/sys_arch.c \
	components/lwip/port/esp8266/freertos/udp_sync.c \
	components/lwip/port/esp8266/netif/ethernetif.c

NEWLIB_SRC := components/newlib/newlib/port/esp_malloc.c \
	components/newlib/newlib/port/esp_newlib.c \
	components/newlib/newlib/port/locks.c \
	components/newlib/newlib/port/select.c \
	components/newlib/newlib/port/syscall.c \
	components/newlib/newlib/port/termios.c \
	components/newlib/newlib/port/time.c

NVS_FLASH_SRC := components/nvs_flash/src/nvs_api.cpp \
	components/nvs_flash/src/nvs_item_hash_list.cpp \
	components/nvs_flash/src/nvs_page.cpp \
	components/nvs_flash/src/nvs_pagemanager.cpp \
	components/nvs_flash/src/nvs_storage.cpp \
	components/nvs_flash/src/nvs_types.cpp

SPI_FLASH_SRC := components/spi_flash/src/partition.c \
	components/spi_flash/src/spi_flash_raw.c \
	components/spi_flash/src/spi_flash.c

TCPIP_ADAPTER_SRC := components/tcpip_adapter/tcpip_adapter_lwip.c

UTIL_SRC := components/util/src/aes.c \
	components/util/src/base64.c \
	components/util/src/crc.c \
	components/util/src/sha.c

WPA_SUPPLICANT_SRC := components/wpa_supplicant/port/os_xtensa.c \
	components/wpa_supplicant/src/crypto/aes-cbc.c \
	components/wpa_supplicant/src/crypto/aes-internal-dec.c \
	components/wpa_supplicant/src/crypto/aes-internal-enc.c \
	components/wpa_supplicant/src/crypto/aes-internal.c \
	components/wpa_supplicant/src/crypto/aes-unwrap.c \
	components/wpa_supplicant/src/crypto/aes-wrap.c \
	components/wpa_supplicant/src/crypto/bignum.c \
	components/wpa_supplicant/src/crypto/crypto_internal-modexp.c \
	components/wpa_supplicant/src/crypto/dh_group5.c \
	components/wpa_supplicant/src/crypto/dh_groups.c \
	components/wpa_supplicant/src/crypto/md5-internal.c \
	components/wpa_supplicant/src/crypto/md5.c \
	components/wpa_supplicant/src/crypto/rc4.c \
	components/wpa_supplicant/src/crypto/sha1-internal.c \
	components/wpa_supplicant/src/crypto/sha1-pbkdf2.c \
	components/wpa_supplicant/src/crypto/sha1.c \
	components/wpa_supplicant/src/crypto/sha256-internal.c \
	components/wpa_supplicant/src/crypto/sha256.c

INC := . \
	components/bootloader_support/include \
	components/freertos/include \
	components/log/include \
	components/lwip/include/lwip/apps \
	components/lwip/lwip/src/include \
	components/lwip/port/esp8266/include \
	components/newlib/newlib/port/include \
	components/nvs_flash/include \
	components/spi_flash/include \
	components/tcpip_adapter/include \
	components/util/include \
	components/wpa_supplicant/include \
	components/wpa_supplicant/port/include

DEF := ESP_PLATFORM \
	IDF_VER=\"v3.2\" \
	__ESP_FILE__=__FILE__ \
	PARTITION_QUEUE_HEADER=\"sys/queue.h\" \
	NVS_CRC_HEADER_FILE=\"crc.h\" \
	__ets__ \
	_GNU_SOURCE

C_CPP_FLAGS := -ffunction-sections \
	-fdata-sections \
	-fstrict-volatile-bitfields \
	-mlongcalls

CFLAGS := -std=gnu99

$(OBJDIR)/components/lwip/lwip/src/core/ipv4/ip4.c.o: CFLAGS += -Wno-implicit-function-declaration
$(OBJDIR)/components/lwip/apps/dhcpserver/dhcpserver.c.o: CFLAGS += -Wno-address

CPPFLAGS :=
AFLAGS :=

LIBDIR := components/esp8266/lib \
	components/newlib/newlib/lib \
	../../../../build/obj/src/hal/ESP8266/ESP8266_RTOS_SDK # Workaround to link libraries from build dir

LIBS_TO_BUILD := esp8266 freertos heap log lwip newlib nvs_flash spi_flash \
	tcpip_adapter util wpa_supplicant

LIB := stdc++ c_nano m core_dbg espnow gcc hal net80211 phy pp_dbg smartconfig ssc \
	wpa wps $(LIBS_TO_BUILD)

LINKED_OBJ :=

# -------------------------

OBJ :=
INC := $(addprefix -I,$(GLOBAL_INC) $(INC))
DEF := $(addprefix -D,$(GLOBAL_DEF) $(DEF))
C_CPP_FLAGS := $(strip $(GLOBAL_C_CPP_FLAGS) $(C_CPP_FLAGS))
CFLAGS := $(strip $(GLOBAL_CFLAGS) $(CFLAGS))
CPPFLAGS := $(strip $(GLOBAL_CPPFLAGS) $(CPPFLAGS))
AFLAGS := $(strip $(GLOBAL_AFLAGS) $(AFLAGS))

# Prevent including this part to main makefile
ifneq ($(ROOT),.)

GET_OBJ = $(patsubst %.c,$(OBJDIR)/%.c.o,$(filter %.c,$(1))) \
	$(patsubst %.cpp,$(OBJDIR)/%.cpp.o,$(filter %.cpp,$(1))) \
	$(patsubst %.s,$(OBJDIR)/%.s.o,$(filter %.s,$(1))) \
	$(patsubst %.S,$(OBJDIR)/%.S.o,$(filter %.S,$(1)))

all: $(patsubst %,$(OBJDIR)/lib%.a,$(LIBS_TO_BUILD))

clean:
	$(call RMDIR,$(OBJDIR))

.SECONDEXPANSION:
$(OBJDIR)/libesp8266.a: $(call GET_OBJ, $(ESP8266_SRC))
	$(AR) cru $@ $^

$(OBJDIR)/libfreertos.a: $(call GET_OBJ, $(FREERTOS_SRC))
	$(AR) cru $@ $^

$(OBJDIR)/libheap.a: $(call GET_OBJ, $(HEAP_SRC))
	$(AR) cru $@ $^

$(OBJDIR)/liblog.a: $(call GET_OBJ, $(LOG_SRC))
	$(AR) cru $@ $^

$(OBJDIR)/liblwip.a: $(call GET_OBJ, $(LWIP_SRC))
	$(AR) cru $@ $^

$(OBJDIR)/libnewlib.a: $(call GET_OBJ, $(NEWLIB_SRC))
	$(AR) cru $@ $^

$(OBJDIR)/libnvs_flash.a: $(call GET_OBJ, $(NVS_FLASH_SRC))
	$(AR) cru $@ $^

$(OBJDIR)/libspi_flash.a: $(call GET_OBJ, $(SPI_FLASH_SRC))
	$(AR) cru $@ $^

$(OBJDIR)/libtcpip_adapter.a: $(call GET_OBJ, $(TCPIP_ADAPTER_SRC))
	$(AR) cru $@ $^

$(OBJDIR)/libutil.a: $(call GET_OBJ, $(UTIL_SRC))
	$(AR) cru $@ $^

$(OBJDIR)/libwpa_supplicant.a: $(call GET_OBJ, $(WPA_SUPPLICANT_SRC))
	$(AR) cru $@ $^

$(OBJDIR)/%.c.o: %.c | $$(@D)/.f
	$(CC) $(DEF) $(INC) $(C_CPP_FLAGS) $(CFLAGS) -MMD -MP -c $< -o $@

$(OBJDIR)/%.cpp.o: %.cpp | $$(@D)/.f
	$(CPP) $(DEF) $(INC) $(C_CPP_FLAGS) $(CPPFLAGS) -MMD -MP -c $< -o $@

$(OBJDIR)/%.s.o: %.s | $$(@D)/.f
	$(AS) $(DEF) $(INC) $(AFLAGS) -MMD -MP -c $< -o $@

$(OBJDIR)/%.S.o: %.S | $$(@D)/.f
	$(AS) $(DEF) $(INC) $(AFLAGS) -MMD -MP -c $< -o $@

%/.f:
	$(call MKDIR,$(dir $@))

-include $(patsubst %.o,%.d,$(OBJ))

endif
