To compile this file use:

Set the PATH to your GNU cross compiler in the Makefile
CROSS_COMPILE= /home/tanzilli/CodeSourcery/Sourcery_G++_Lite/bin/arm-none-eabi-

cd ./AcmeBoot
make CHIP=at91sam9g20 BOARD=NetusG20 ORIGIN=sdcard DESTINATION=sdram OP_BOOTSTRAP_MCI=on TRACE_LEVEL=1 clean all

Program the Netus G20 dataflash with ./bin/boot-NetusG20-sdcard2sdram.bin binary file

Same in the first FAT16 partition the uimage uncompressed kernel file. 
Please note that you have to se the boot option in the kerlen config because the foxg20-bin file in not used anymore.

The default MAC address is hardware coded in the main.c:
/// The MAC address used for demo
static unsigned char MacAddress[6] = {0x00, 0x45, 0x56, 0x78, 0x9a, 0xac};

More info on AcmeBoot is available on http://foxg20.acmesystems.it/doku.php?id=dev:acmeboot



