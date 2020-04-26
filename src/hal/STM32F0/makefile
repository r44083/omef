-include ../../../config.mk

SRC := dma/dma.cpp \
	exti/exti.cpp \
	gpio/gpio.cpp \
	rcc/rcc.cpp \
	spi/spi.cpp \
	systick/systick.cpp \
	tim/tim.cpp \
	uart/uart.cpp \
	wdt/wdt.cpp

INC :=

DEF :=
C_CPP_FLAGS := -Wno-unused-variable
CFLAGS :=
CPPFLAGS :=
AFLAGS :=

LIBDIR :=
LIB :=
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
