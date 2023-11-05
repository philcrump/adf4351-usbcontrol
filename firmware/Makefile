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

SOURCES = $(filter-out $(BINARY).c, $(wildcard *.c))

OBJS = $(SOURCES:.c=.o)

V=1
all: bin

stlink-flash: $(BINARY).bin
	@printf "  FLASH  $<\n"
	$(STFLASH) write $(BINARY).bin 0x8000000

include ./rules.mk
