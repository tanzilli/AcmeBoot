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
#ifdef ORIGIN_serialflash

//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------

#include <board.h>
#include <board_memories.h>
#include <pio/pio.h>
#include <utility/assert.h>
#include <utility/trace.h>
#include <spi-flash/at26d.h>
#include "main.h"

//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------

// Transfer return codes
#define BOOT_AT26_SUCCESS   0 /// All requested transfer are successfull
#define BOOT_AT26_NO_DEVICE 1 /// No AT26 devices has been detected

#define SPCK        1000000 /// SPI clock frequency, in Hz.

//------------------------------------------------------------------------------
//         Definitions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//         Internal variables
//------------------------------------------------------------------------------
Spid spid; /// SPI driver instance.
At26 at26; /// AT26 driver instance.

//------------------------------------------------------------------------------
//         Internal functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// Initialize AT46 driver. Depending on the AT91SAM device, SLOTA or SLOTB are
/// defined according to the board.h definitions.
//------------------------------------------------------------------------------
void AT26Init()
{
    // Initialize the SPI transfer, spid and at26 parameters
    // As there is no serial flash on AT91-EK boards, they are usually connected through
    // an extension board which has the SD card format. That's why AT26 pin definition
    // is used.
#if defined(AT26_SLOT_A)
    const Pin pins[]  = {BOARD_AT26_A_SPI_PINS, BOARD_AT26_A_NPCS_PIN};
    PIO_Configure(pins, PIO_LISTSIZE(pins));
    SPID_Configure(&spid, BOARD_AT26_A_SPI_BASE, BOARD_AT26_A_SPI_ID);
    AT26_Configure(&at26, &spid, BOARD_AT26_A_NPCS);
#elif defined(AT26_SLOT_B)
    const Pin pins[]  = {BOARD_AT26_B_SPI_PINS, BOARD_AT26_B_NPCS_PIN};
    PIO_Configure(pins, PIO_LISTSIZE(pins));
    SPID_Configure(&spid, BOARD_AT26_B_SPI_BASE, BOARD_AT26_B_SPI_ID);
    AT26_Configure(&at26, &spid, BOARD_AT26_B_NPCS);
#else
#error "-F- AT26_SLOT_A or  AT26_SLOT_B must be defined"
#endif //BOARD_AT26_A
}

//------------------------------------------------------------------------------
//         Exported functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// Initialize serial flash devices and transfer one or sevral modules from
/// serial flash to the target memory (SRAM/SDRAM).
/// \param pTd    Pointer to transfer descriptor array.
/// \param nbTd  Number of transfer descriptors.
/// \return 0 if successfull.
//------------------------------------------------------------------------------
int BOOT_AT26_CopyBin(const Tdesc *pTd, unsigned char nbTd)
{
    unsigned int jedecId;
    unsigned int sizeToCopy; // remaining bytes number to copy
    unsigned int pageSize;  // Dataflash page size
    unsigned int packetSize; // Dataflash read size
    unsigned int memoryOffset; // Dataflash read offset
    unsigned char *pDest; // Dest pointer for copy

    // Initialize the SPI transfer, spid and at26 parameters
    AT26Init();
    TRACE_INFO("Init AT26 Serialflash\n\r");

    // Read the JEDEC ID of the device to identify it
    jedecId = AT26D_ReadJedecId(&at26);

    if (AT26_FindDevice(&at26, jedecId) == 0) {
        TRACE_ERROR("Device not detected or unknown\n\r");
        return BOOT_AT26_NO_DEVICE;
    }

    pageSize = AT26_PageSize(&at26);

    if (at26.pDesc->name)
        TRACE_INFO("SerialFlash detected : %s\n\r", at26.pDesc->name);

    // Foreach module transfer data from AT26 to memory
    while (nbTd--) {

        TRACE_INFO("Copy \"%s\" (%d bytes) from SerialFlash 0x%08x to 0x%08x\n\r", 
                      pTd->strDescr, 
                      pTd->size,
                      pTd->offset,
                      (unsigned int) pTd->dest
                      );

        pDest = (unsigned char*)pTd->dest;
        sizeToCopy = pTd->size;
        memoryOffset = pTd->offset;

        while (sizeToCopy > 0) {

            // Write packet after packets
            packetSize = (sizeToCopy < pageSize) ? sizeToCopy : pageSize;

            // Read the page and copy
            AT26D_Read(&at26, pDest, packetSize, memoryOffset);

            // Update pointer
            memoryOffset += packetSize;
            pDest += packetSize;
            sizeToCopy -= packetSize;
        }

        ++pTd;
    }
    return BOOT_AT26_SUCCESS;
}

#endif // FROM_serialflash

