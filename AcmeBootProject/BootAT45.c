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

#ifdef ORIGIN_dataflash

//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------

#include <board.h>
#include <board_memories.h>
#include <pio/pio.h>
#include <utility/assert.h>
#include <utility/trace.h>
#include <utility/math.h>
#include <spi-flash/at45d.h>

#include "main.h"

//------------------------------------------------------------------------------
//         Definitions
//------------------------------------------------------------------------------

// Transfer return codes
#define BOOT_AT45_SUCCESS   0 /// All requested transfer are successfull
#define BOOT_AT45_NO_DEVICE 1 /// No AT45 devices has been detected

#define SPCK        20000000 /// SPI clock frequency, in Hz.

//------------------------------------------------------------------------------
//         Internal variables
//------------------------------------------------------------------------------

Spid spid; /// SPI driver instance.
At45 at45; /// AT45 driver instance.

//------------------------------------------------------------------------------
//         Internal functions
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
/// Initialize AT45 driver. Depending on the AT91SAM device, SLOTA or SLOTB are
/// defined according to the board.h definitions.
//------------------------------------------------------------------------------
static void AT45Init(void)
{
#if defined(AT45_SLOT_A)
    const Pin pins[]  = {BOARD_AT45_A_SPI_PINS, BOARD_AT45_A_NPCS_PIN};
    PIO_Configure(pins, PIO_LISTSIZE(pins));
    SPID_Configure(&spid, BOARD_AT45_A_SPI_BASE, BOARD_AT45_A_SPI_ID);
    SPID_ConfigureCS(&spid, BOARD_AT45_A_NPCS, AT45_CSR(BOARD_MCK, SPCK));
    AT45_Configure(&at45, &spid, BOARD_AT45_A_NPCS);
#elif defined(AT45_SLOT_B)
    const Pin pins[]  = {BOARD_AT45_B_SPI_PINS, BOARD_AT45_B_NPCS_PIN};
    PIO_Configure(pins, PIO_LISTSIZE(pins));
    SPID_Configure(&spid, BOARD_AT45_B_SPI_BASE, BOARD_AT45_B_SPI_ID);
    SPID_ConfigureCS(&spid, BOARD_AT45_B_NPCS, AT45_CSR(BOARD_MCK, SPCK));
    AT45_Configure(&at45, &spid, BOARD_AT45_B_NPCS);
#else
    #error "-F- AT45_SLOT_A or  AT45_SLOT_B must be defined"
#endif //AT45_slot_A    
}

//------------------------------------------------------------------------------
//         Exported functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// Erase the first page of the dataflash memory. This function allows to
/// auto erase the at91bootstrap in order to restart from SAM-BA boot atfter
/// the next power-up sequence.
//------------------------------------------------------------------------------
void BOOT_AT45_EraseBoot(void)
{
    // Initialize the SPI transfer, spid and at45 parameters
    AT45Init();
    TRACE_INFO("Init AT45 Dataflash\n\r");
    
    // Erase the first page of the dataflash
    AT45D_Erase(&at45, 0);
}

//------------------------------------------------------------------------------
/// Initialize AT45 devices and transfer one or sevral modules from At45 to the
/// target memory (SRAM/SDRAM).
/// \param pTd    Pointer to transfer descriptor array.
/// \param nbTd  Number of transfer descriptors.
/// \return 0 if successfull.
//------------------------------------------------------------------------------
int BOOT_AT45_CopyBin(const Tdesc *pTd, unsigned char nbTd)
{
    const At45Desc *pDesc; // Pointer to the connected AT45 parameter
    unsigned int pageSize;  // Dataflash page size
    unsigned int packetSize; // Dataflash read size
    unsigned int memoryOffset; // Dataflash read offset
    unsigned char *pDest; // Dest pointer for copy
    unsigned int sizeToCopy; // remaining bytes number to copy
    unsigned char status = 0;

    // Initialize the SPI transfer, spid and at45 parameters
    AT45Init();
    TRACE_INFO("Init AT45 Dataflash\n\r");

    // Configure pins
    status = AT45D_GetStatus(&at45);

    // Detect AT45 device connected
    pDesc = AT45_FindDevice(&at45, status);

    if (pDesc == (const At45Desc *) 0) {
        TRACE_ERROR("Device not detected or unknown\n\r");
        return BOOT_AT45_NO_DEVICE;
    }

    pageSize = AT45_PageSize(&at45);

    if (at45.pDesc->name)
        TRACE_INFO("Dataflash detected : %s\n\r", at45.pDesc->name);

    // Foreach module transfer data from AT45 to memory
    while (nbTd--) {

        TRACE_INFO("Copy \"%s\" (%d bytes) from DataFlash 0x%08x to 0x%08x\n\r", 
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
            AT45D_Read(&at45, pDest, packetSize, memoryOffset);

            // Update pointer
            memoryOffset += packetSize;
            pDest += packetSize;
            sizeToCopy -= packetSize;
        }

        ++pTd;
    }

    return BOOT_AT45_SUCCESS;
}

#endif // dataflash
