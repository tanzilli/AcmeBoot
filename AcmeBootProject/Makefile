# ----------------------------------------------------------------------------
#         ATMEL Microcontroller Software Support
# ----------------------------------------------------------------------------
# Copyright (c) 2008, Atmel Corporation
#
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# - Redistributions of source code must retain the above copyright notice,
# this list of conditions and the disclaimer below.
# 
# Atmel's name may not be used to endorse or promote products derived from
# this software without specific prior written permission. 
# 
# DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
# DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# ---------------------------------------------------------------------------- 

#     Makefile for compiling at91bootstrap-project


dataflash:
	export RUN=1
	make CHIP=at91sam9g20 BOARD=NetusG20 \
		ORIGIN=sdcard DESTINATION=sdram OP_BOOTSTRAP_MCI=on \
		FLASH_MEMORY=data_flash DRAM_SIZE=DRAM_64MB TRACE_LEVEL=1 \
		clean all

serialflash:
	export RUN=1
	make CHIP=at91sam9g20 BOARD=NetusG20 \
		ORIGIN=sdcard DESTINATION=sdram OP_BOOTSTRAP_MCI=on \
		FLASH_MEMORY=serial_flash DRAM_SIZE=DRAM_64MB TRACE_LEVEL=1 \
		clean all

#-------------------------------------------------------------------------------
#        User-modifiable options
#-------------------------------------------------------------------------------

# Optimization level, put in comment for debugging
ifeq ($(OP_BOOTSTRAP), on)
OPTIMIZATION = -march=armv5t -Os
else
    ifeq ($(OP_BOOTSTRAP_MCI), on)
    OPTIMIZATION = -march=armv5t -Os
    else

    OPTIMIZATION = -O2
    endif
endif

# AT91 library directory
AT91LIB = ../at91lib
EXTLIBS = ../external_libs

# Output directories
BIN = bin
OBJ = obj

#-------------------------------------------------------------------------------
#        Command lines parameters managment
#-------------------------------------------------------------------------------

# CHIP        (M)  chip name
# BOARD       (M)  board name
# ORIGIN      (M)  memory where the module will be read
# DESTINATION (M)  memory where the module will be copied
# BIN_SIZE    (M)  module size to read and copy
# FROM_ADDR   (M*) source memory address offset where the module will be read
# FILE_NAME   (M*) name of the file to copy (for FAT formated memory)
# DEST_ADDR   (M)  destination memory address where the module will be copied
# VDDMEMSEL   (O)  EBI power supply selection (1.8V, 3.3V)
# STR_DESCR   (O)  Module description (optional user data)
# BOOTNAME    (O)  Bootstrap binary name
# TRACE_LEVEL (O)  if not defined, none trace
# ORIGIN_REF  (M*) Needed for EEPROM boot

# FLASH_MEMORY (M) Memory type on Netus module (data_flash or serial_flash)
# DRAM_SIZE    (M) DRAM size on Netus module (DRAM_64MB or DRAM_128MB)

# Notes :
# (M) mandatory parameter
# (O) optional parameter
# (M*) mandatory parameter depending on the boot (SDCard boot or other)

# Default output file basename (if not defined by user)
#-------------------------------------------------------------------------------
BOOTNAME_BASE = boot-$(BOARD)-$(ORIGIN)2$(DESTINATION)

ifndef    FPGAINIT
FPGAINIT = 0
endif

ifeq ($(FPGAINIT),1)
BOOTNAME = $(BOOTNAME_BASE)-fpga
else
BOOTNAME = $(BOOTNAME_BASE)
endif

# if there is only one binary to copy, its size, its source address and its 
# destination address can be given as makefile argument
#-------------------------------------------------------------------------------
ifdef BIN_SIZE
CFLAGS += -DBIN_SIZE=$(BIN_SIZE)
endif

ifdef FROM_ADDR
CFLAGS += -DFROM_ADDR=$(FROM_ADDR)
endif

ifdef DEST_ADDR
CFLAGS += -DDEST_ADDR=$(DEST_ADDR)
endif

ifdef FILE_NAME
CFLAGS += -DFILE_NAME=$(FILE_NAME)
endif

ifdef STR_DESCR
CFLAGS += -DSTR_DESCR=$(STR_DESCR)
endif

# EBI power supply : EBI1V8 = 1.8V or EBI3V3 = 3.3V
#-------------------------------------------------------------------------------
ifeq ($(VDDMEMSEL),EBI1V8)
CFLAGS += -DVDDMEMSEL_EBI1V8
endif

ifeq ($(VDDMEMSEL),EBI3V3)
CFLAGS += -DVDDMEMSEL_EBI3V3
endif

#ifdef VDDMEMSEL
#CFLAGS += -DVDDMEMSEL=$(VDDMEMSEL)
#endif

# Compile with chip specific features
#-------------------------------------------------------------------------------
ifdef $(RUN)  #avoid error if called with dataflash or serialflash
include $(AT91LIB)/boards/$(BOARD)/$(CHIP)/chip.mak
endif 

# Special init for CAP9
#-------------------------------------------------------------------------------
ifeq ($(CHIP),at91cap9)
ifeq ($(FPGAINIT),1)
CFLAGS += -DFPGAINIT
endif
endif

# PATH_BOOT_H allows to use a special boot.h for specific project with several 
# modules to copy. If not defined, the normal boot.h of the at91bootstrap 
# project is used
#-------------------------------------------------------------------------------
ifndef PATH_BOOT_H
PATH_BOOT_H = .
endif

# PATH_OUTPUT_DIR allows to choose the directory where the BIN directory 
# will be created
#-------------------------------------------------------------------------------
ifdef PATH_OUTPUT_DIR
BIN = $(PATH_OUTPUT_DIR)/bin
OBJ = $(PATH_OUTPUT_DIR)/obj
endif

#-------------------------------------------------------------------------------
#        Tools
#-------------------------------------------------------------------------------

# Tool suffix when cross-compiling
#CROSS_COMPILE=arm-linux-gnueabi-
CROSS_COMPILE= /home/$(USER)/CodeSourcery/Sourcery_G++_Lite/bin/arm-none-eabi-

# Compilation tools
CC = $(CROSS_COMPILE)gcc
SIZE = $(CROSS_COMPILE)size
STRIP = $(CROSS_COMPILE)strip
OBJCOPY = $(CROSS_COMPILE)objcopy

CFLAGS += -Wall -mlong-calls -ffunction-sections
CFLAGS += -g $(OPTIMIZATION) $(INCLUDES) 
CFLAGS += -D$(CHIP) 
CFLAGS += -DBOARD=$(BOARD)
CFLAGS += -DORIGIN_$(ORIGIN)
CFLAGS += -DDESTINATION_$(DESTINATION)
CFLAGS += -DTRACE_LEVEL=$(TRACE_LEVEL)

ASFLAGS = -g $(OPTIMIZATION) $(INCLUDES) -D$(CHIP) -D__ASSEMBLY__
LDFLAGS = -g $(OPTIMIZATION) -nostartfiles -Wl,--gc-sections,-Map=$(OUTPUT).map,--cref

#-------------------------------------------------------------------------------
#        Files
#-------------------------------------------------------------------------------

# Includes paths
INCLUDES += -I$(PATH_BOOT_H)
INCLUDES += -I$(AT91LIB)
INCLUDES += -I$(AT91LIB)/boards/$(BOARD)
INCLUDES += -I$(AT91LIB)/peripherals
INCLUDES += -I$(AT91LIB)/components
INCLUDES += -I$(AT91LIB)/memories
INCLUDES += -I$(EXTLIBS)/fat/fatfs/src
INCLUDES += -I$(AT91LIB)/drivers

# Sources path
VPATH += $(AT91LIB)/utility
VPATH += $(AT91LIB)/peripherals
VPATH += $(AT91LIB)/peripherals/pio
VPATH += $(AT91LIB)/memories/spi-flash
VPATH += $(AT91LIB)/peripherals/irq
VPATH += $(AT91LIB)/peripherals/dbgu
#VPATH += $(AT91LIB)/components/ethernet/dm9161
VPATH += $(AT91LIB)/boards/$(BOARD)
VPATH += $(AT91LIB)/boards/$(BOARD)/$(CHIP)
VPATH += $(AT91LIB)/peripherals/cp15
VPATH += $(AT91LIB)/peripherals/rstc
#VPATH += $(AT91LIB)/drivers/macb

# Objects built from C source files
C_OBJECTS += main.o
C_OBJECTS += pio.o
#C_OBJECTS += emac.o
#C_OBJECTS += macb.o
#C_OBJECTS += dm9161.o
C_OBJECTS += rstc.o
C_OBJECTS += aic.o
C_OBJECTS += board_memories.o
C_OBJECTS += board_lowlevel.o
C_OBJECTS += stdio.o
C_OBJECTS += string.o
C_OBJECTS += dbgu.o
C_OBJECTS += trace.o

ifeq ($(CHIP),at91cap9)
ifeq ($(FPGAINIT),1)
C_OBJECTS += board_fpga_init.o
endif
endif

# Objects built from Assembly source files
ASM_OBJECTS += board_cstartup.o
ASM_OBJECTS += cp15_asm_gcc.o

#-------------------------------------------------------------------------------
# MEMORY SOURCE : SDcard
#-------------------------------------------------------------------------------
ifeq ($(ORIGIN),sdcard)

INCLUDES += -I.

VPATH += $(AT91LIB)/memories
VPATH += $(AT91LIB)/utility
VPATH += $(AT91LIB)/memories/sdmmc
VPATH += $(AT91LIB)/peripherals/mci
VPATH += $(EXTLIBS)/fat/fatfs/src
VPATH += $(AT91LIB)/peripherals/dma
VPATH += $(AT91LIB)/drivers/dmad

C_OBJECTS += BootSdcard.o
C_OBJECTS += diskio.o
C_OBJECTS += ff.o
C_OBJECTS += Media.o
C_OBJECTS += MEDSdcard.o
C_OBJECTS += spid.o
C_OBJECTS += math.o

ifeq ($(FLASH_MEMORY), data_flash)
C_OBJECTS += at45.o
endif

ifeq ($(FLASH_MEMORY), serial_flash)
C_OBJECTS += at26.o
endif

ifeq ($(CHIP_IP_MCI), MCI_DMA)
C_OBJECTS += dmad.o
C_OBJECTS += dma.o
C_OBJECTS += mci_hs.o
else
C_OBJECTS += mci.o
endif
C_OBJECTS += sdmmc_mci.o

MEMORIES = sram

endif

#-------------------------------------------------------------------------------
# DRAM memory size on NetusG20
#-------------------------------------------------------------------------------

ifeq ($(DRAM_SIZE), DRAM_64MB)
CFLAGS += -DDRAM_64MB
endif

ifeq ($(DRAM_SIZE), DRAM_128MB)
CFLAGS += -DDRAM_128MB
endif

#-------------------------------------------------------------------------------
# Flash memory type on NetusG20
#-------------------------------------------------------------------------------

ifeq ($(FLASH_MEMORY), data_flash)
CFLAGS += -DDATA_FLASH
endif

ifeq ($(FLASH_MEMORY), serial_flash)
CFLAGS += -DSERIAL_FLASH
endif


#-------------------------------------------------------------------------------
# Optimization bootstrap : at91sam9260
#-------------------------------------------------------------------------------
ifeq ($(OP_BOOTSTRAP), on)
C_INTERWORK_FLAGS += $(CFLAGS)
C_INTERWORK_FLAGS += -mthumb-interwork
CFLAGS += -mthumb
CFLAGS += -DOP_BOOTSTRAP_$(OP_BOOTSTRAP)
endif

ifeq ($(OP_BOOTSTRAP), on)
C_INTERWORK_OBJECTS = cp15.o
else
    ifeq ($(OP_BOOTSTRAP_MCI), on)
    C_INTERWORK_OBJECTS = cp15.o
    else
    C_OBJECTS += cp15.o
    endif
endif

#-------------------------------------------------------------------------------
# Optimize bootstrap mci re-init : at91sam9263 at91sam9g20
#-------------------------------------------------------------------------------
ifeq ($(OP_BOOTSTRAP_MCI), on)
CFLAGS += -DOP_BOOTSTRAP_MCI_$(OP_BOOTSTRAP_MCI)
endif

ifeq ($(OP_BOOTSTRAP_MCI), on)
C_INTERWORK_FLAGS += $(CFLAGS)
C_INTERWORK_FLAGS += -mthumb-interwork
CFLAGS += -mthumb
CFLAGS += -DOP_BOOTSTRAP_MCI_$(OP_BOOTSTRAP_MCI)
endif

#===============================================================================

OUTPUT      := $(BIN)/$(BOOTNAME)

#-------------------------------------------------------------------------------
#		Rules
#-------------------------------------------------------------------------------
# Append OBJ and BIN directories to filenames
C_OBJECTS   := $(addprefix $(OBJ)/, $(C_OBJECTS))
ASM_OBJECTS := $(addprefix $(OBJ)/, $(ASM_OBJECTS))
C_INTERWORK_OBJECTS   := $(addprefix $(OBJ)/, $(C_INTERWORK_OBJECTS))

all: $(BIN) $(OBJ) $(MEMORIES)

$(BIN) $(OBJ):
	@mkdir $@

$(MEMORIES): $(ASM_OBJECTS) $(C_OBJECTS) $(C_INTERWORK_OBJECTS)
	echo "**************************************************************************"
	echo "***** Boot from $(ORIGIN) to $(DESTINATION) (execute in $(MEMORIES)) *****"
	echo "--------------------------------------------------------------------------"
	$(CC) $(LDFLAGS) -T"$(AT91LIB)/boards/$(BOARD)/$(CHIP)/$(MEMORIES).lds" -o $(OUTPUT).elf $^
	$(OBJCOPY) -O binary $(OUTPUT).elf $(OUTPUT).bin
	$(SIZE) $(ASM_OBJECTS) $(C_OBJECTS) $(OUTPUT).elf

$(C_OBJECTS): $(OBJ)/%.o: %.c Makefile
	$(CC) $(CFLAGS) -c -o $@ $<

$(C_INTERWORK_OBJECTS): $(OBJ)/%.o: %.c Makefile
	$(CC) $(C_INTERWORK_FLAGS) -c -o $@ $<

$(ASM_OBJECTS): $(OBJ)/%.o: %.S Makefile
	$(CC) $(ASFLAGS) -c -o $@ $<
	
debug_sram: sram
	perl ../resources/gdb/debug.pl $(OUTPUT).elf	

clean:
	-rm -f $(OBJ)/*.o $(BIN)/*.elf $(BIN)/*.map

help:
	@echo  "make dataflash"
	@echo  "   Compile AcmeBoot for NetusG20 with dataflash"
	@echo  "make serialflash"
	@echo  "   Compile AcmeBoot for NetusG20 with serialflash"

#	@echo  "Below are the original command line parameters by Atmel:"
#	@echo  "   - CHIP        Chip name (Ex: at91cap9)"
#	@echo  "   - BOARD       board name (Ex: at91cap9-dk)"
#	@echo  "   - ORIGIN      Source memory where the module to copy will read"
#	@echo  "                 (dataflash / serialflash / sdcard / nandflash / norflash)"
#	@echo  "   - DESTINATION Destination memory where the module will be copied"
#	@echo  "                 (sdram / ddram)"
#	@echo  "   - BIN_SIZE    Binary size to copy"
#	@echo  "   - FROM_ADDR   Address in source memory where the module will be read"
#	@echo  "                 (not used for SDCard boot)"
#	@echo  "   - FILE_NAME   file name to copy (only for SDcard boot)"
#	@echo  "   - DEST_ADDR   Address in destination memory where the module will be copied"
#	@echo  "                 and where the bootstrap program will do a jump"
#	@echo  "   - STR_DESCR   Description of the module to copy"
#	@echo  "   - BOOTNAME    File name of the output binary. By default the output name is"
#	@echo  "                 boot-$(BOARD)-$(ORIGIN)2$(DESTINATION).bin"
#	@echo  "   - TRACE_LEVEL Determine the trace level. If not present, none trace."
#	@echo ""
#	@echo " Note 1: The use of FROM_ADDR and FILE_NAME is exclusive."
#	@echo ""
#	@echo " Note 2: the last 3 parameters are optional. All the other are mandatory."

