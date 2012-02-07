/* ----------------------------------------------------------------------------
 * AcmeBoot - Bootload utility for Netus G20 
 *
 * Sergio Tanzilli - tanzilli@acmesystems.it
 * Roberto Asquini - asquini@acmesystems.it
 * Luca Pascarella - pascarella@acmesystems.it
 * Claudio Mignanti - mignanti@acmesystems.it
 * Antonio Galea - antonio.galea@gmail.com
 *
 * http://www.acmesystems.it
 * http://foxg20.acmesystems.it/doku.php?id=dev:acmeboot
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ACME SYSTEMS "AS IS" IN THE SAME 
 * TERMS OF THE ORIGINAL ATMEL DISCLAIMER LISTED BELOW.
 * ----------------------------------------------------------------------------
 */

/* ----------------------------------------------------------------------------
 * AcmeBoot is based on Atmel AT91Bootstrap v.3.0 
 * ----------------------------------------------------------------------------
 */

/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2008, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

#define ACME_BOOTSTRAP_VERSION "1.20"

/* ----------------------------------------------------------------------------
 * CHANGELOG
 * ----------------------------------------------------------------------------
 *
 * 1.20 Propagate SD_Stop() to work well with any microSD brand
 *      Deleted the code to read watchdog.txt because too large to be
 *      fixed in 16KB. 
 *
 * 1.19 Watchdog timer enabled
 *
 * 1.18 Read the Kernel CMDLINE configuration from the file cmdline.txt
 *      on the first microSD partition
 *
 *      Read the MACHTYPE from the file machtype.txt
 *      on the first microSD partition
 *
 * 1.17 Led red blinks again (like the < 1.16 version ) when 
 *      the microSD is not found
 * 
 * 1.16 Tested with Data and Serial Flash (256KB and 8MB)
 *      First experiment with 128MB od DRAM
 *
 * 1.15 Configure PC10 and PC5 as GPIO instead of A25
 *      Compatible with Atmel AT45DB021D 256K Dataflash 
 *
 * 1.14 Configure PC4 and PC5 as GPIO instead of A23 and A24
 *
 * 1.13 Reset the PHY chip at startup
 * 	    Revised code styling
 * 
 * 1.12 Red led (PC7) on at reset and off if an uimage is found on the microSD
 *      When an error occour the red led blink with these meanings:
 * 
 *      2 blinks: uimage not found on microSD
 * 
 * 1.11 DPI port bux fix
 * ----------------------------------------------------------------------------
 */

//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------

#include <board.h>
#include <board_memories.h>
#include <pio/pio.h>
#include <irq/irq.h>
#include <cp15/cp15.h>
#include <dbgu/dbgu.h>
#include <utility/assert.h>
#include <utility/trace.h>
#include <utility/math.h>
#include <memories/spi-flash/at26.h>
#include <spi-flash/at45.h>
#include <peripherals/rstc/rstc.h>
#include <peripherals/mci/mci.h>
#include <string.h>

#include "main.h"
#include <boot.h>

static struct {
	// The Magic number is used used by the Pizzica ISP utility to change on-the-fly the
	// factory default MAC address and by the AcmeBoot to understand when loaded from ISP 
	// or from serial/data flash
	unsigned int MyMagicNumber;

	// MAC address
	unsigned char MacAddress[6];
} mm = {
	0x5C5C5C5C,
	{0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC}
};

// The PINs' on PHY reset
static const Pin emacRstPins[] = {BOARD_EMAC_RST_PINS};

// Red led
static const Pin foxg20_red_led = {1 << 7, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT};

// Defines of PC4, PC5 and PC10 (new on version 1.14)
static const Pin foxg20_pc4  = {1 <<  4, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_INPUT, PIO_PULLUP};
static const Pin foxg20_pc5  = {1 <<  5, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_INPUT, PIO_PULLUP};
static const Pin foxg20_pc10 = {1 << 10, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_INPUT, PIO_PULLUP};

//------------------------------------------------------------------------------
/// Error coding on red led
//------------------------------------------------------------------------------

#define UIMAGE_NOT_FOUND   2 	// uimage on microSD not found
#define FLASH_WRITE_ERROR  3	// flash writing error
#define MICROSD_NOT_FOUND  4 	// microSD not found
	
static void blink_delay(void)
{
	int volatile delay_counter=0;

	while (delay_counter<1000000) 
		delay_counter++;
}

static void led_error(int blink_code)
{
	int blink_counter;
	int i;

	for (;;) {	
		for (blink_counter=0;blink_counter<blink_code;blink_counter++) {
			PIO_Clear(&foxg20_red_led);
			blink_delay();
			PIO_Set(&foxg20_red_led);			
			blink_delay();
		}
		for (i=0;i<10;i++) blink_delay();
	}   
}

//------------------------------------------------------------------------------
/// Jump to the first address to execute application
//------------------------------------------------------------------------------
static void GoToJumpAddress(unsigned int jumpAddr, unsigned int matchType,unsigned int kernelParams)
{
	typedef void(*fctType)(volatile unsigned int, volatile unsigned int, volatile unsigned int);
	void(*pFct)(volatile unsigned int r0_val, volatile unsigned int r1_val, volatile unsigned int r2_val);

	pFct = (fctType)jumpAddr;
	pFct(0/*dummy value in r0*/, matchType/*matchType in r1*/, kernelParams/*kernelParams in r2*/);

	while(1); //never reach
}

#define be32_to_cpu(a) ((a)[0] << 24 | (a)[1] << 16 | (a)[2] << 8 | (a)[3])
#define SDRAM_START 0x20000000
#define KERNEL_UIMAGE	"uImage"
#define KERNEL_CMDLINE	"cmdline.txt"
#define MACH_TYPE_FILE 	"machtype.txt"
#define WATCHDOG_FILE 	"watchdog.txt"

//------------------------------------------------------------------------------
//         Internal definitions
//------------------------------------------------------------------------------

#define MAXPAGESIZE     1056

/// Page buffer.
static unsigned char pBuffer[MAXPAGESIZE];

#ifdef SERIAL_FLASH
/// Maximum device page size in bytes.
#define SPI_BASE    AT91C_BASE_SPI0             // Address of the SPI peripheral connected to the AT26.
#define SPI_ID      AT91C_ID_SPI0               // Peripheral identifier of the SPI connected to the AT26.
#define SPI_CS      1                           // Chip select value used to select the AT26 chip.
#define SPI_PINS    PINS_SPI0, PIN_SPI0_NPCS1   // SPI peripheral pins to configure to access the serial flash.
#endif


#ifdef DATA_FLASH
/// SPI clock frequency, in Hz.
#define SPCK        1000000
#define BOARD_AT45_SPI_BASE           AT91C_BASE_SPI0
#define BOARD_AT45_SPI_ID             AT91C_ID_SPI0
#define BOARD_AT45_SPI_PINS           PINS_SPI0
#define BOARD_AT45_NPCS_PIN           PIN_SPI0_NPCS1
#define BOARD_AT45_NPCS               1
#endif

//------------------------------------------------------------------------------
//         Internal variables
//------------------------------------------------------------------------------

/// SPI driver instance.
static Spid spid;

#ifdef SERIAL_FLASH
/// Serial flash driver instance.
static At26 at26;

/// Pins to configure for the application.
static Pin pins[] = {SPI_PINS};
#endif

#ifdef DATA_FLASH
/// AT45 driver instance.
static At45 at45;

/// Pins used by the application.
static const Pin pins[]  = {BOARD_AT45_SPI_PINS, BOARD_AT45_NPCS_PIN};
#endif

//------------------------------------------------------------------------------
//         Internal functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// Interrupt service routine for the SPI peripheral. Forwards the interrupt
/// to the SPI driver.
//------------------------------------------------------------------------------
static void ISR_Spi(void)
{
	SPID_Handler(&spid);
}


#ifdef DATA_FLASH
//------------------------------------------------------------------------------
/// Retrieves and returns the At45 current status, or 0 if an error
/// happened.
/// \param pAt45  Pointer to a At45 driver instance.
//------------------------------------------------------------------------------
static unsigned char AT45_GetStatus(At45 *pAt45)
{
	unsigned char error;
	unsigned char status;

	// Sanity checks
	//ASSERT(pAt45, "-F- AT45_GetStatus: pAt45 is null\n\r");

	// Issue a status register read command
	error = AT45_SendCommand(pAt45, AT45_STATUS_READ, 1, &status, 1, 0, 0, 0);
	//ASSERT(!error, "-F- AT45_GetStatus: Failed to issue command.\n\r");

	// Wait for command to terminate
	while (AT45_IsBusy(pAt45));

	return status;
}

//------------------------------------------------------------------------------
/// Waits for the At45 to be ready to accept new commands.
/// \param pAt45  Pointer to a At45 driver instance.
//------------------------------------------------------------------------------
static void AT45_WaitReady(At45 *pAt45) 
{
	unsigned char ready = 0;

	// Sanity checks
	//ASSERT(pAt45, "-F- AT45_WaitUntilReady: pAt45 is null\n\r");

	// Poll device until it is ready
	while (!ready) {
		ready = AT45_STATUS_READY(AT45_GetStatus(pAt45));
	}
}

//------------------------------------------------------------------------------
/// Reads and returns the JEDEC identifier of a At45.
/// \param pAt45  Pointer to a At45 driver instance.
//------------------------------------------------------------------------------
static unsigned int AT45_GetJedecId(At45 *pAt45)
{
	unsigned char error;
	unsigned int id;

	// Sanity checks
	//ASSERT(pAt45, "-F- AT45_GetJedecId: pAt45 is null\n\r");

	// Issue a manufacturer and device ID read command
	error = AT45_SendCommand(pAt45, AT45_ID_READ, 1, (void *) &id, 4, 0, 0, 0);
	//ASSERT(!error, "-F- AT45_GetJedecId: Could not issue command.\n\r");

	// Wait for transfer to finish
	while (AT45_IsBusy(pAt45));

	return id;
}

//------------------------------------------------------------------------------
/// Reads data from the At45 inside the provided buffer. Since a continuous
/// read command is used, there is no restriction on the buffer size and read
/// address.
/// \param pAt45  Pointer to a At45 driver instance.
/// \param pBuffer  Data buffer.
/// \param size  Number of bytes to read.
/// \param address  Address at which data shall be read.
//------------------------------------------------------------------------------
static void AT45_Read(
	At45 *pAt45,
	unsigned char *pBuffer,
	unsigned int size,
	unsigned int address) 
{
	unsigned char error;

	// Sanity checks
	//ASSERT(pAt45, "-F- AT45_Read: pAt45 is null\n\r");

	//ASSERT(pBuffer, "-F- AT45_Read: pBuffer is null\n\r");

	// Issue a continuous read array command
	error = AT45_SendCommand(pAt45, AT45_CONTINUOUS_READ_LEG, 8, pBuffer, size, address, 0, 0);
	//ASSERT(!error, "-F- AT45_Read: Failed to issue command\n\r");

	// Wait for the read command to execute
	while (AT45_IsBusy(pAt45));
}

//------------------------------------------------------------------------------
/// Writes data on the At45 at the specified address. Only one page of
/// data is written that way; if the address is not at the beginning of the
/// page, the data is written starting from this address and wraps around to
/// the beginning of the page.
/// \param pAt45  Pointer to a At45 driver instance.
/// \param pBuffer  Buffer containing the data to write.
/// \param size  Number of bytes to write.
/// \param address  Destination address on the At45.
//------------------------------------------------------------------------------
static void AT45_Write(
	At45 *pAt45,
	unsigned char *pBuffer,
	unsigned int size,
	unsigned int address) 
{
	unsigned char error;

	// Sanity checks
	//ASSERT(pAt45, "-F- AT45_Write: pAt45 is null.\n\r");

	//ASSERT(pBuffer, "-F- AT45_Write: pBuffer is null.\n\r");

	//ASSERT(size <= pAt45->pDesc->pageSize, "-F- AT45_Write: Size too big\n\r");

	// Issue a page write through buffer 1 command
	error = AT45_SendCommand(pAt45, AT45_PAGE_WRITE_BUF1, 4, pBuffer, size, address, 0, 0);
	//ASSERT(!error, "-F- AT45_Write: Could not issue command.\n\r");

	// Wait until the command is sent
	while (AT45_IsBusy(pAt45));

	// Wait until the At45 becomes ready again
	AT45_WaitReady(pAt45);
}

//------------------------------------------------------------------------------
/// Erases a page of data at the given address in the At45.
/// \param pAt45  Pointer to a At45 driver instance.
/// \param address  Address of page to erase.
//------------------------------------------------------------------------------
static void AT45_Erase(At45 *pAt45, unsigned int address) 
{
	unsigned char error;

	// Sanity checks
	//ASSERT(pAt45, "-F- AT45_Erase: pAt45 is null\n\r");

	// Issue a page erase command.
	error = AT45_SendCommand(pAt45, AT45_PAGE_ERASE, 4, 0, 0, address, 0, 0);
	//ASSERT(!error, "-F- AT45_Erase: Could not issue command.\n\r");

	// Wait for end of transfer
	while (AT45_IsBusy(pAt45));

	// Poll until the At45 has completed the erase operation
	AT45_WaitReady(pAt45);
}
#endif


#ifdef SERIAL_FLASH
//------------------------------------------------------------------------------
/// Reads and returns the status register of the serial flash.
/// \param pAt26  Pointer to an AT26 driver instance.
//------------------------------------------------------------------------------
static unsigned char AT26_ReadStatus(At26 *pAt26)
{
	unsigned char error, status;

	SANITY_CHECK(pAt26);

	// Issue a status read command
	error = AT26_SendCommand(pAt26, AT26_READ_STATUS, 1, &status, 1, 0, 0, 0);

	// Wait for transfer to finish
	while (AT26_IsBusy(pAt26));

	return status;
}

//------------------------------------------------------------------------------
/// Writes the given value in the status register of the serial flash device.
/// \param pAt26  Pointer to an AT26 driver instance.
/// \param status  Status to write.
//------------------------------------------------------------------------------
static void AT26_WriteStatus(At26 *pAt26, unsigned char status)
{
	unsigned char error;

	SANITY_CHECK(pAt26);

	// Issue a write status command
	error = AT26_SendCommand(pAt26, AT26_WRITE_STATUS, 1, &status, 1, 0, 0, 0);

	while (AT26_IsBusy(pAt26));
}

//------------------------------------------------------------------------------
/// Waits for the serial flash device to become ready to accept new commands.
/// \param pAt26  Pointer to an AT26 driver instance.
//------------------------------------------------------------------------------
static void AT26_WaitReady(At26 *pAt26)
{
	unsigned char ready = 0;

	SANITY_CHECK(pAt26);

	// Read status register and check busy bit
	while (!ready) {
		ready = ((AT26_ReadStatus(pAt26) & AT26_STATUS_RDYBSY) == AT26_STATUS_RDYBSY_READY);
	}
}

//------------------------------------------------------------------------------
/// Reads and returns the serial flash device ID.
/// \param pAt26  Pointer to an AT26 driver instance.
//------------------------------------------------------------------------------
static unsigned int AT26_ReadJedecId(At26 *pAt26)
{
	unsigned char error;
	unsigned int id = 0;

	SANITY_CHECK(pAt26);

	// Issue a read ID command
	error = AT26_SendCommand(pAt26, AT26_READ_JEDEC_ID, 1,
		             (unsigned char *) &id, 3, 0, 0, 0);

	// Wait for transfer to finish
	while (AT26_IsBusy(pAt26));

	return id;
}

//------------------------------------------------------------------------------
/// Enables critical writes operation on a serial flash device, such as sector
/// protection, status register, etc.
/// \para pAt26  Pointer to an AT26 driver instance.
//------------------------------------------------------------------------------
static void AT26_EnableWrite(At26 *pAt26)
{
	unsigned char error;

	SANITY_CHECK(pAt26);

	// Issue a write enable command
	error = AT26_SendCommand(pAt26, AT26_WRITE_ENABLE, 1, 0, 0, 0, 0, 0);

	// Wait for end of transfer
	while (AT26_IsBusy(pAt26));
}

//------------------------------------------------------------------------------
/// Unprotects the contents of the serial flash device.
/// Returns 0 if the device has been unprotected; otherwise returns
/// SF_PROTECTED.
/// \param pAt26  Pointer to an AT26 driver instance.
//------------------------------------------------------------------------------
static unsigned char AT26_Unprotect(At26 *pAt26)
{
	unsigned char status;

	SANITY_CHECK(pAt26);

	// Get the status register value to check the current protection
	status = AT26_ReadStatus(pAt26);
	if ((status & AT26_STATUS_SWP) == AT26_STATUS_SWP_PROTNONE) {
		// Protection already disabled
		return 0;
	}

	// Check if sector protection registers are locked
	if ((status & AT26_STATUS_SPRL) == AT26_STATUS_SPRL_LOCKED) {
		// Unprotect sector protection registers by writing the status reg.
		AT26_EnableWrite(pAt26);
		AT26_WriteStatus(pAt26, 0);
	}

	// Perform a global unprotect command
	AT26_EnableWrite(pAt26);
	AT26_WriteStatus(pAt26, 0);

	// Check the new status
	if ((status & (AT26_STATUS_SPRL | AT26_STATUS_SWP)) != 0) {
		return AT26_ERROR_PROTECTED;
	} else {
		return 0;
	}
}

//------------------------------------------------------------------------------
/// Erases all the content of the memory chip.
/// \param pAt26  Pointer to an AT26 driver instance.
//------------------------------------------------------------------------------
static unsigned char AT26_EraseChip(At26 *pAt26)
{
	unsigned char status;
	unsigned char error;

	SANITY_CHECK(pAt26);

	// Check that the flash is ready an unprotected
	status = AT26_ReadStatus(pAt26);
	if ((status & AT26_STATUS_SWP) != AT26_STATUS_SWP_PROTNONE) {
		TRACE_WARNING("Device protected\n\r");
		return AT26_ERROR_PROTECTED;
	}

	// Enable critical write operation
	AT26_EnableWrite(pAt26);

	// Erase the chip
	error = AT26_SendCommand(pAt26, AT26_CHIP_ERASE_2, 1, 0, 0, 0, 0, 0);
	//ASSERT(!error, "%d\n\r",__LINE__);

	while (AT26_IsBusy(pAt26));    
	AT26_WaitReady(pAt26);

	return 0;
}

//------------------------------------------------------------------------------
/// Erases the specified 4KB block of the serial firmware dataflash.
/// Returns 0 if successful; otherwise returns AT26_ERROR_PROTECTED if the
/// device is protected or AT26_ERROR_BUSY if it is busy executing a command.
/// \param pAt26  Pointer to an AT26 driver instance.
/// \param address  Address of the block to erase.
//------------------------------------------------------------------------------
static unsigned char AT26_EraseBlock(At26 *pAt26, unsigned int address)
{
	unsigned char status;
	unsigned char error;

	SANITY_CHECK(pAt26);

	// Check that the flash is ready an unprotected
	status = AT26_ReadStatus(pAt26);
	if ((status & AT26_STATUS_RDYBSY) != AT26_STATUS_RDYBSY_BUSY) {
		TRACE_WARNING("Device not ready\n\r");
		return AT26_ERROR_BUSY;
	} else if ((status & AT26_STATUS_SWP) != AT26_STATUS_SWP_PROTNONE) {
		TRACE_WARNING("Device protected\n\r");
		return AT26_ERROR_PROTECTED;
	}

	// Enable critical write operation
	AT26_EnableWrite(pAt26);

	// Start the block erase command
	error = AT26_SendCommand(pAt26, AT26_BLOCK_ERASE_4K, 4, 0, 0, address, 0, 0);
	//ASSERT(!error, "%d\n\r",__LINE__);

	while (AT26_IsBusy(pAt26));
	AT26_WaitReady(pAt26);

	return 0;
}

//------------------------------------------------------------------------------
/// Writes data at the specified address on the serial firmware dataflash. The
/// page(s) to program must have been erased prior to writing. This function
/// handles page boundary crossing automatically.
/// Returns 0 if successful; otherwise, returns AT26_ERROR_PROGRAM is there has
/// been an error during the data programming.
/// \param pAt26  Pointer to an AT26 driver instance.
/// \param pData  Data buffer.
/// \param size  Number of bytes in buffer.
/// \param address  Write address.
//------------------------------------------------------------------------------
static unsigned char AT26_Write(
	At26 *pAt26,
	unsigned char *pData,
	unsigned int size,
	unsigned int address)
{
	unsigned int pageSize;
	unsigned int writeSize;
	unsigned char error;
	unsigned char status;

	SANITY_CHECK(pAt26);
	SANITY_CHECK(pData);

	// Retrieve device page size
	pageSize = AT26_PageSize(&at26);

	// Program one page after the other
	while (size > 0) {
		// Compute number of bytes to program in page
		writeSize = min(size, pageSize - (address % pageSize));
		
		// Enable critical write operation
		AT26_EnableWrite(pAt26);

		// Program page
		error = AT26_SendCommand(pAt26, AT26_BYTE_PAGE_PROGRAM, 4,
					pData, writeSize, address, 0, 0);
		//ASSERT(!error, "%d\n\r",__LINE__);
		while (AT26_IsBusy(pAt26));
		AT26_WaitReady(pAt26);

		// Make sure that write was without error
		status = AT26_ReadStatus(pAt26);
		if ((status & AT26_STATUS_EPE) == AT26_STATUS_EPE_ERROR) {
			return AT26_ERROR_PROGRAM;
		}
		size -= writeSize;
		address += writeSize;
	}

	return 0;
}

//------------------------------------------------------------------------------
/// Reads data from the specified address on the serial flash.
/// \param pAt26  Pointer to an AT26 driver instance.
/// \param pData  Data buffer.
/// \param size  Number of bytes to read.
/// \param address  Read address.
//------------------------------------------------------------------------------
static void AT26_Read(
	At26 *pAt26,
	unsigned char *pData,
	unsigned int size,
	unsigned int address)
{
	unsigned char error;

	// Start a read operation
	error = AT26_SendCommand(pAt26, AT26_READ_ARRAY_LF, 4, pData, size, address, 0, 0);
	//ASSERT(!error, "%d\n\r",__LINE__);
	while (AT26_IsBusy(pAt26));
}
#endif

//------------------------------------------------------------------------------
/// Bootstrap main application.
/// Transfer data from media to main memory and return the next application entry
/// point address.
//------------------------------------------------------------------------------
int main()
{
	unsigned char *sram_address = (unsigned char *)0x200000;
	unsigned int numPages;
	unsigned int pageSize;
	unsigned int page;

	char *tmp;
	char mach_type_buffer[5];
        char watchdog_buffer;
	unsigned int mach_type_number;

	#ifdef SERIAL_FLASH
	unsigned int jedecId;
	unsigned int j;
	unsigned int flash_address;
	#endif

	#ifdef DATA_FLASH
	unsigned char original;
	unsigned char copy;
	const At45Desc *pDesc;
	int i;
	#endif

	PIO_Configure(&foxg20_red_led, 1);

	// Configure PC4, PC5 and PC10 as GPIO (new on version 1.14)
	PIO_Configure(&foxg20_pc4, 1);
	PIO_Configure(&foxg20_pc5, 1);
	PIO_Configure(&foxg20_pc10, 1);

	PIO_Clear(&foxg20_red_led);			

	// Save the length of SRAM (16K) on the 6th vector. In this way when this
	// code is saved on Data or Serial flash is managed as a boot file
	// from Atmel RomBOOT

	*((unsigned int *)(sram_address + 0x14)) = (unsigned int)16*1024;

	//-------------------------------------------------------------------------
	// Configure traces
	//-------------------------------------------------------------------------
	TRACE_CONFIGURE_ISP(DBGU_STANDARD, 115200, BOARD_MCK);

	printf("\n\rAcmeBoot %s\n\r", ACME_BOOTSTRAP_VERSION);
	printf("MCK = %dMHz\n\r", (int)(BOARD_MCK/1000000));

	// If the RTC registers are unitializated set then to default 
	// value of 20 aug 2010 11:49
	if (*(unsigned int *)0xFFFFFD50==0) { 
		*(unsigned int *)0xFFFFFD50=0x4C6E6BB4;
	}

	//-------------------------------------------------------------------------
	// Enable I-Cache
	//-------------------------------------------------------------------------
	CP15_EnableIcache();

	//-------------------------------------------------------------------------
	// Configure external RAM where the application will be transfered
	//-------------------------------------------------------------------------

	// SDRAM
	#if defined(DESTINATION_sdram)
	//printf("Init SDRAM\n\r");
	BOARD_ConfigureSdram(BOARD_SDRAM_BUSWIDTH);
	#endif

	#ifdef DATA_FLASH
	// Configure pins
	PIO_Configure(pins, PIO_LISTSIZE(pins));

	// SPI and At45 driver initialization
	IRQ_ConfigureIT(BOARD_AT45_SPI_ID, 0, ISR_Spi);
	SPID_Configure(&spid, BOARD_AT45_SPI_BASE, BOARD_AT45_SPI_ID);
	SPID_ConfigureCS(&spid, BOARD_AT45_NPCS, AT45_CSR(BOARD_MCK, SPCK));
	AT45_Configure(&at45, &spid, BOARD_AT45_NPCS);
	IRQ_EnableIT(BOARD_AT45_SPI_ID);

	// Identify the At45 device
	pDesc = 0;
	while (!pDesc) {
		pDesc = AT45_FindDevice(&at45, AT45_GetStatus(&at45));
	}
	//printf("%s found\n\r", at45.pDesc->name);

	// Output JEDEC identifier of device
	printf("id: 0x%08X\n\r", AT45_GetJedecId(&at45));

	// Get device parameters
	numPages = AT45_PageNumber(&at45);
	pageSize = AT45_PageSize(&at45);
	#endif

	#ifdef SERIAL_FLASH
	//-------------------------------------------------------------------------
	// Initialize the SPI and serial flash
	//-------------------------------------------------------------------------

	PIO_Configure(pins, PIO_LISTSIZE(pins));
	IRQ_ConfigureIT(SPI_ID, 0, ISR_Spi);
	SPID_Configure(&spid, SPI_BASE, SPI_ID);
	AT26_Configure(&at26, &spid, SPI_CS);
	IRQ_EnableIT(SPI_ID);

	// Read the JEDEC ID of the device to identify it
	jedecId = AT26_ReadJedecId(&at26);
	if (AT26_FindDevice(&at26, jedecId)) {
		printf("%s found\n\r", AT26_Name(&at26));
	} else {
		printf("Dev unknown\n\r");
		for (;;);
	}
	//ASSERT(MAXPAGESIZE >= AT26_PageSize(&at26), "-F- MAXPAGESIZE too small\n\r");

	// Get device parameters
	numPages = AT26_PageNumber(&at26);
	pageSize = AT26_PageSize(&at26);

	//Unprotected the flash
	AT26_Unprotect(&at26);

	printf("Pg #: %d\n\r",numPages);
	printf("Pg s: %d\n\r",pageSize);

	#endif

	// Check the magic number to know if has been downloaded from flash
	// or from Pizzica

	if (mm.MyMagicNumber!=0x12345678) {
		// Set the magic number
		mm.MyMagicNumber=0x12345678;

		// Erase the first 16K of dataflash
		 printf("Erasing\n\r");

		#ifdef DATA_FLASH
		for (page=0;page<(16384/pageSize+1);page++) {
			AT45_Erase(&at45, page * AT45_PageSize(&at45));
		}
		#endif

		#ifdef SERIAL_FLASH
		AT26_EraseChip(&at26);
		#endif

		// Here the AcmeBoot copy itself from the SRAM0 to the flash memory

		#ifdef DATA_FLASH
		for (page=0;page<(16384/pageSize+1);page++) {
			memcpy(pBuffer, sram_address + page * AT45_PageSize(&at45), AT45_PageSize(&at45));

			//printf("Write page %d\n\r", page);
			AT45_Write(&at45, pBuffer, AT45_PageSize(&at45), page * AT45_PageSize(&at45));
			AT45_Read (&at45, pBuffer, AT45_PageSize(&at45), page * AT45_PageSize(&at45));

			for (i=0;i<AT45_PageSize(&at45);i++) {
				original = *(sram_address + page * AT45_PageSize(&at45) + i);
				copy     = *(pBuffer+i);
				if (original != copy) {
					printf ("WR:%02X RD:%02X\n\r",original,copy);
					led_error(FLASH_WRITE_ERROR);
					// This point is never reached
				}
			}

		}
		#endif

		#ifdef SERIAL_FLASH
		flash_address = 0;
		for (page=0;page<(16384/pageSize+1);page++) {
			// Fill buffer
			for (j=0; j < pageSize; j++) {
				pBuffer[j] = *(sram_address+flash_address+j);
			}
			// Write buffer
			AT26_Write(&at26, pBuffer, pageSize, flash_address);

			// Read page back and check result
			memset(pBuffer, 0, pageSize);
			AT26_Read(&at26, pBuffer, pageSize, flash_address);

			for (j=0;j<pageSize;j++) {
				if (pBuffer[j] != *(sram_address+flash_address+j)) {
					printf("WR:%02X RD:%02X\n\r",*(sram_address+flash_address+j), pBuffer[j]);
					led_error(FLASH_WRITE_ERROR);
					// This point is never reached
				}
			}
			flash_address += pageSize;
		}
		#endif
	}

	//-------------------------------------------------------------------------
	// Set up the MAC address storing it on the MACB CPU register
	// EMAC_SA1L and EMAC_SA1H
	//-------------------------------------------------------------------------

	printf("MAC: %02X%02X%02X%02X%02X%02X\n\r",
		mm.MacAddress[0], mm.MacAddress[1], mm.MacAddress[2],
		mm.MacAddress[3], mm.MacAddress[4], mm.MacAddress[5]);

	// Power ON
	AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_EMAC;

	// Disable TX & RX and more
	AT91C_BASE_EMACB->EMAC_NCR = 0;

	// disable 
	AT91C_BASE_EMACB->EMAC_IDR = ~0;

	// Set the MAC address
	AT91C_BASE_EMACB->EMAC_SA1L = ( ((unsigned int)mm.MacAddress[3] << 24)
		                 | ((unsigned int)mm.MacAddress[2] << 16)
		                 | ((unsigned int)mm.MacAddress[1] << 8 )
		                 |                mm.MacAddress[0] );

	AT91C_BASE_EMACB->EMAC_SA1H = ( ((unsigned int)mm.MacAddress[5] << 8 )
		                 |                mm.MacAddress[4] );

	AT91C_BASE_EMACB->EMAC_NCR = AT91C_EMAC_CLRSTAT;

	// Configure PINS
	PIO_Configure(emacRstPins, PIO_LISTSIZE(emacRstPins));

	// Execute reset
	RSTC_SetExtResetLength(7312);
	RSTC_ExtReset();

	// Wait for end hardware reset
	while (!RSTC_GetNrstLevel());

	// Enable User Reset
	AT91C_BASE_RSTC->RSTC_RMR |= AT91C_RSTC_URSTEN | (0xA5<<24);

	if (Acme_SDcard_Init()!=0) {
	        printf("No microSD\n\r");
		led_error(MICROSD_NOT_FOUND);
	}

	//--------------------------------------------------------------------
	// Read the command line parameters
	//--------------------------------------------------------------------
	//
	// Author: Antonio Galea, Sergio Tanzilli, Claudio Mignanti
	// Ideas borrowed from
	// http://www.simtec.co.uk/products/SWLINUX/files/booting_article.html
	//--------------------------------------------------------------------

#define ATAG_POS        SDRAM_START+0x100
#define CMDLINE_POS	ATAG_POS+8*4	
#define CMDLINE_LEN	0x400

#define ATAG_NONE	0x00000000
#define ATAG_CORE	0x54410001
#define ATAG_MEM	0x54410002
#define ATAG_CMDLINE	0x54410009

	// We won't read more than 1k; let's make sure string will be null-terminated
	memset((void *)CMDLINE_POS,0,CMDLINE_LEN+1);
	
	if (Acme_SDcard_CopyFile(KERNEL_CMDLINE,CMDLINE_POS,CMDLINE_LEN)==0) {
		tmp = (char *)CMDLINE_POS;
		int size = strlen(tmp);
		unsigned int *addr = ATAG_POS;

		// ATAG_CORE	
		*addr++ = 2;
		*addr++ = ATAG_CORE;

		// ATAG_MEM	
		*addr++ = 4;
		*addr++ = ATAG_MEM;
		*addr++ = 0x4000000;   // 64Mb of memory
		*addr++ = SDRAM_START; // Physical SDRAM start address

		// ATAG_CMDLINE
		*addr++ = 2 + ((size + 3) / 4);
		*addr++ = ATAG_CMDLINE;
		addr += ((size + 3) / 4);

		// ATAG_NONE
		*addr++ = 2;
		*addr = ATAG_NONE;
	} 

	//--------------------------------------------------------------------
	// Read the MACH_TYPE from machtype.txt
	//
	// 3129 = acmenetusfoxg20. Used starting from Kernel 2.6.38
	// 1624 = at91sam9g20ek. Previous machtype
	//
	// See ARM machine registry on:
	// http://www.arm.linux.org.uk/developer/machines/
	//--------------------------------------------------------------------

	mach_type_number = 1624;
	if (Acme_SDcard_CopyFile(MACH_TYPE_FILE,(unsigned char *)mach_type_buffer,(unsigned long)4)==0) {
		mach_type_buffer[4]=0;
		mach_type_number=(unsigned int)atoi(mach_type_buffer);
	}
	//printf("machtype=%d\n\r",mach_type_number);


	// Enable the watchdog timer        
	AT91C_BASE_WDTC->WDTC_WDMR = AT91C_WDTC_WDDIS;

	//--------------------------------------------------------------------
	// Read the Kernel image cutting the first 64 of header
	//--------------------------------------------------------------------

	if (Acme_SDcard_CopyFile(KERNEL_UIMAGE,SDRAM_START+0x8000-0x40,0)!=0) {
		printf("%s not found\n\r",KERNEL_UIMAGE);
		led_error(UIMAGE_NOT_FOUND);
		// This point is never reached
	}


	// Red led off
	PIO_Clear(&foxg20_red_led);

        //SW Reset of SD card reader
        Acme_SDcard_Stop();

	printf("Run Kernel\n\r");

	GoToJumpAddress(SDRAM_START+0x8000, mach_type_number, (unsigned int *)ATAG_POS);

	led_error(FLASH_WRITE_ERROR);
	return 0;
}

