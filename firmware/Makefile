##

BINARY = main

LDSCRIPT = ./stm32f103xb.ld

OPENCM3_DIR = ./libopencm3

LIBNAME		= opencm3_stm32f1
DEFS		+= -DSTM32F1

FP_FLAGS	?= -msoft-float
ARCH_FLAGS	= -mthumb -mcpu=cortex-m3 $(FP_FLAGS) -mfix-cortex-m3-ldrd

################################################################################
# OpenOCD specific variables

OOCD		?= openocd
OOCD_INTERFACE	?= flossjtag
OOCD_TARGET	?= stm32f1x

################################################################################
# Black Magic Probe specific variables
# Set the BMP_PORT to a serial port and then BMP is used for flashing
BMP_PORT	?=

################################################################################
# texane/stlink specific variables
#STLINK_PORT	?= :4242

TARGETS		:= stm32/f1

V=1
all: bin check-libopencm3

check-libopencm3:
	@if [ ! "`ls -A $(OPENCM3_DIR)`" ] ; then \
		printf "######## ERROR ########\n"; \
		printf "\tlibopencm3 is not initialized.\n"; \
		printf "\tPlease run:\n"; \
		printf "\t$$ git submodule init\n"; \
		printf "\t$$ git submodule update\n"; \
		printf "\tbefore running make.\n"; \
		printf "######## ERROR ########\n"; \
		exit 1; \
	fi
	@if [ ! -f "$(OPENCM3_DIR)/lib/lib$(LIBNAME).a" ]; then \
		echo "INFO: Need to compile libopecm3"; \
		$(Q)$(MAKE) -C $(OPENCM3_DIR) TARGETS=$(TARGETS); \
	fi

stlink-flash: $(BINARY).bin
	@printf "  FLASH  $<\n"
	$(STFLASH) write $(BINARY).bin 0x8000000

include ./rules.mk
