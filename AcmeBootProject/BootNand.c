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

#ifdef ORIGIN_nandflash

//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------

#include <board.h>
#include <board_memories.h>
#include <pio/pio.h>
#include <utility/assert.h>
#include <utility/trace.h>
#include "../at91lib/memories/nandflash/SkipBlockNandFlash.h"
#include "../at91lib/memories/nandflash/RawNandFlash.h"

#include <string.h>
#include "main.h"

//------------------------------------------------------------------------------
//         Definitions
//------------------------------------------------------------------------------

// Transfer return codes
#define BOOT_NAND_SUCCESS            0 /// All requested transfer are successfull
#define BOOT_NAND_ERROR_NO_DEVICE    1 /// No nand devices has been detected
#define BOOT_NAND_ERROR_GP           2

//------------------------------------------------------------------------------
//         Internal variables
//------------------------------------------------------------------------------

/// Nandflash memory size.
static unsigned int memSize;
/// Size of one block in the nandflash, in bytes.
static unsigned int blockSize;
/// Number of blocks in nandflash.
static unsigned short numBlocks;
/// Size of one page in the nandflash, in bytes.
static unsigned short pageSize;
/// Number of page per block
static unsigned short numPagesPerBlock;
// Nandflash bus width
static unsigned char nfBusWidth = 16;

#ifdef PINS_NANDFLASH

/// Pins used to access to nandflash.
static const Pin pPinsNf[] = {PINS_NANDFLASH};
/// Nandflash device structure.
static struct SkipBlockNandFlash skipBlockNf; 
/// Address for transferring command bytes to the nandflash.
static unsigned int cmdBytesAddr = BOARD_NF_COMMAND_ADDR;
/// Address for transferring address bytes to the nandflash.
static unsigned int addrBytesAddr = BOARD_NF_ADDRESS_ADDR;
/// Address for transferring data bytes to the nandflash.
static unsigned int dataBytesAddr = BOARD_NF_DATA_ADDR;
/// Nandflash chip enable pin.
static const Pin nfCePin = BOARD_NF_CE_PIN;
/// Nandflash ready/busy pin.
static const Pin nfRbPin = BOARD_NF_RB_PIN;

#else

/// Pins used to access to nandflash.
static const Pin pPinsNf[] = {{0, 0, 0, 0, 0}}; 
/// Nandflash device structure.
static struct SkipBlockNandFlash skipBlockNf; 
/// Address for transferring command bytes to the nandflash.
static unsigned int cmdBytesAddr = 0;
/// Address for transferring address bytes to the nandflash.
static unsigned int addrBytesAddr = 0;
/// Address for transferring data bytes to the nandflash.
static unsigned int dataBytesAddr = 0;
/// Nandflash chip enable pin.
static const Pin nfCePin = {0, 0, 0, 0, 0};
/// Nandflash ready/busy pin.
static const Pin nfRbPin = {0, 0, 0, 0, 0};

#endif

//------------------------------------------------------------------------------
//         Internal functions
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
/// Initialize NAND flash driver. 
//------------------------------------------------------------------------------
static void NandInit()
{
    //-------------------------------------------------------------------------
    TRACE_INFO("Init NAND Flash\n\r");        
    
    // Configure SMC for Nandflash accesses (done each time because of old ROM codes)
    BOARD_ConfigureNandFlash(nfBusWidth);
    PIO_Configure(pPinsNf, PIO_LISTSIZE(pPinsNf));
    
    //memset(&skipBlockNf, 0, sizeof(skipBlockNf));          
 
    if (SkipBlockNandFlash_Initialize(&skipBlockNf,
                                              0,
                                              cmdBytesAddr,
                                              addrBytesAddr,
                                              dataBytesAddr,
                                              nfCePin,
                                              nfRbPin)) {
    
        TRACE_ERROR("Device Unknown\n\r");
    } 
    
    // Check the data bus width of the NandFlash
    nfBusWidth = NandFlashModel_GetDataBusWidth((struct NandFlashModel *)&skipBlockNf);
    
    // Reconfigure bus width
    BOARD_ConfigureNandFlash(nfBusWidth);

    TRACE_INFO("Nandflash driver initialized\n\r"); 

    // Get device parameters
    memSize = NandFlashModel_GetDeviceSizeInBytes(&skipBlockNf.ecc.raw.model);
    blockSize = NandFlashModel_GetBlockSizeInBytes(&skipBlockNf.ecc.raw.model);
    numBlocks = NandFlashModel_GetDeviceSizeInBlocks(&skipBlockNf.ecc.raw.model);
    pageSize = NandFlashModel_GetPageDataSize(&skipBlockNf.ecc.raw.model);
    numPagesPerBlock = NandFlashModel_GetBlockSizeInPages(&skipBlockNf.ecc.raw.model);
    
    TRACE_INFO("Size of the whole device in bytes : 0x%x \n\r",memSize);
    TRACE_INFO("Size in bytes of one single block of a device : 0x%x \n\r",blockSize);
    TRACE_INFO("Number of blocks in the entire device : 0x%x \n\r",numBlocks);
    TRACE_INFO("Size of the data area of a page in bytes : 0x%x \n\r",pageSize);
    TRACE_INFO("Number of pages in the entire device : 0x%x \n\r",numPagesPerBlock);
    TRACE_INFO("Bus width : %d \n\r",nfBusWidth);
}

//------------------------------------------------------------------------------
//         Exported functions
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
/// Erase the first page of the dataflash memory. This function allows to
/// auto erase the at91bootstrap in order to restart from SAM-BA boot atfter
/// the next power-up sequence.
//------------------------------------------------------------------------------
void BOOT_NAND_EraseBoot()
{
    // Initialize the Nand parameters
    NandInit();
    
    // Erase the first page of the nandflash
    #if !defined (OP_BOOTSTRAP_on)
    SkipBlockNandFlash_EraseBlock(&(skipBlockNf), 0, SCRUB_ERASE);
    #else
    RawNandFlash_EraseBlock(&(skipBlockNf.ecc.raw), 0);
    #endif
}


//------------------------------------------------------------------------------
/// Initialize NAND devices and transfer one or sevral modules from Nand to the
/// target memory (SRAM/SDRAM).
/// \param pTd    Pointer to transfer descriptor array.
/// \param nbTd   Number of transfer descriptors.
//------------------------------------------------------------------------------
int BOOT_NAND_CopyBin(const Tdesc *pTd, unsigned char nbTd)
{ 
    unsigned short block;
    unsigned short page;
    unsigned short offset;    
   
    unsigned char *ptr;
    unsigned byteRead;    
    
    // Errors returned by SkipNandFlash functions
    unsigned char error = 0;    
/*...........................................................................*/ 
    
    // Initialize Nand
    NandInit();
    // Transfert data from Nand to External RAM
    //-------------------------------------------------------------------------   

    // Foreach module transfer data from Nand to memory
    while (nbTd--) {
       
        TRACE_INFO("Copy \"%s\" (%d bytes) from NAND 0x%08x to 0x%08x\n\r", 
                      pTd->strDescr, 
                      pTd->size, 
                      pTd->offset, 
                      pTd->dest
                      );    
    
        #if !defined(OP_BOOTSTRAP_on)
        // Check word alignment
        if (pTd->offset % 4) {
    
              TRACE_ERROR("Offset not word aligned\n\r");
              return BOOT_NAND_ERROR_GP;
        }
        #endif
    
        #if !defined(OP_BOOTSTRAP_on)
        // Retrieve page and block addresses        
        if (NandFlashModel_TranslateAccess(&(skipBlockNf.ecc.raw.model),
                                           pTd->offset,
                                           pTd->size,
                                           &block,
                                           &page,
                                           &offset)) {
              TRACE_ERROR("TranslateAccess Error\n\r");
              return BOOT_NAND_ERROR_GP;
        }
        #else
        NandFlashModel_TranslateAccess(&(skipBlockNf.ecc.raw.model),
                                           pTd->offset,
                                           pTd->size,
                                           &block,
                                           &page,
                                           &offset);
        #endif
    
        #if !defined(OP_BOOTSTRAP_on)
        if (page || offset) {
    
              TRACE_ERROR("Address is not a block start\n\r");
              return BOOT_NAND_ERROR_GP;
        }    
        #endif

        ptr = (unsigned char*)pTd->dest;
        byteRead = pTd->size;
        
        while(block < numBlocks)
        {
              for(page = 0; page < numPagesPerBlock; page++) {

                  do{
                      error = SkipBlockNandFlash_ReadPage(&skipBlockNf, block, page, ptr, 0);
                      if (error == NandCommon_ERROR_BADBLOCK)
                          block++;
                      else
                          break;
                  }while(block < numBlocks && page == 0);

                  #if !defined(OP_BOOTSTRAP_on)
                  if (error) {
                      TRACE_ERROR("SkipBlockNandFlash_ReadBlock: Cannot read page %d of block %d.\n\r", page, block);
                      return BOOT_NAND_ERROR_GP;
                  }
                  #endif
                  ptr += pageSize;
                
                  if(byteRead <= pageSize) {
                      byteRead = 0;
                      break;
                  }
                  else {
                      byteRead -= pageSize;
                  }
              }
          
              if(byteRead == 0) {
                break;
            }          
            else {
                block++;        
            }
        }
        
        ++pTd;
    }

    return BOOT_NAND_SUCCESS;
}

#endif //nandflash
