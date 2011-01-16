#!/bin/bash

#make CHIP=at91sam9g20 BOARD=NetusG20 ORIGIN=sdcard DESTINATION=sdram OP_BOOTSTRAP_MCI=on TRACE_LEVEL=1 all
python pizzica.py -f bin/boot-NetusG20-sdcard2sdram.bin -d /dev/ttyUSB0

