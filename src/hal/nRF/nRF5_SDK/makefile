-include ../../../../config.mk

SRC := modules/nrfx/mdk/gcc_startup_nrf52.S \
	modules/nrfx/mdk/system_nrf52.c \
	modules/nrfx/drivers/src/nrfx_clock.c \
	integration/nrfx/legacy/nrf_drv_clock.c \
	components/libraries/util/app_util_platform.c \
	\
	external/freertos/portable/CMSIS/nrf52/port_cmsis_systick.c \
	external/freertos/portable/CMSIS/nrf52/port_cmsis.c \
	external/freertos/portable/GCC/nrf52/port.c \
	external/freertos/source/croutine.c \
	external/freertos/source/event_groups.c \
	external/freertos/source/list.c \
	external/freertos/source/queue.c \
	external/freertos/source/stream_buffer.c \
	external/freertos/source/tasks.c \
	external/freertos/source/timers.c \
	external/freertos/source/portable/Common/mpu_wrappers.c \
	external/freertos/source/portable/MemMang/heap_4.c

INC := external/freertos/config \
	external/freertos/portable/CMSIS/nrf52 \
	external/freertos/portable/GCC/nrf52 \
	external/freertos/source/include \
	modules/nrfx/drivers/include \
	modules/nrfx/hal \
	integration/nrfx/legacy \
	components/libraries/log \
	components/libraries/experimental_section_vars \
	components/libraries/log/src

DEF := __STARTUP_CLEAR_BSS __START=main

C_CPP_FLAGS :=

CFLAGS :=
CPPFLAGS :=
AFLAGS :=

LIBDIR := components/toolchain/cmsis/dsp/GCC
LIB := arm_cortexM4lf_math

LINKED_OBJ :=

# -------------------------

OBJ := $(patsubst %.c,$(OBJDIR)/%.c.o,$(filter %.c,$(SRC))) \
	$(patsubst %.cpp,$(OBJDIR)/%.cpp.o,$(filter %.cpp,$(SRC))) \
	$(patsubst %.s,$(OBJDIR)/%.s.o,$(filter %.s,$(SRC))) \
	$(patsubst %.S,$(OBJDIR)/%.S.o,$(filter %.S,$(SRC)))

INC := $(addprefix -I,$(GLOBAL_INC) $(INC))
DEF := $(addprefix -D,$(GLOBAL_DEF) $(DEF))
C_CPP_FLAGS := $(strip $(GLOBAL_C_CPP_FLAGS) $(C_CPP_FLAGS))
CFLAGS := $(strip $(GLOBAL_CFLAGS) $(CFLAGS))
CPPFLAGS := $(strip $(GLOBAL_CPPFLAGS) $(CPPFLAGS))
AFLAGS := $(strip $(GLOBAL_AFLAGS) $(AFLAGS))

# Prevent including this part to main makefile
ifneq ($(ROOT),.)

all: $(OBJ)

clean:
	$(call RMDIR,$(OBJDIR))

.SECONDEXPANSION:
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