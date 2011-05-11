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

#ifndef MAIN_H_
#define MAIN_H_

//-----------------------------------------------------------------------------
//         Headers
//-----------------------------------------------------------------------------

#define BOOTSTRAP_VERSION "3.0"

//-----------------------------------------------------------------------------
//         Macros
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
//         Types
//-----------------------------------------------------------------------------

typedef struct _Tdesc {
    unsigned int   offset;   /// Memory offset for non FAT formated memory
    const char    *fileName; /// File Name for FAT formated memory (SDCard)    
    unsigned int   dest;     /// Destination address
    unsigned int   size;     /// Module size in bytes
    const char    *strDescr; /// Module description (optional, can be null pointer)
} Tdesc;

#define TDESC_LISTSIZE(list) (sizeof(list)/ sizeof(Tdesc))

//------------------------------------------------------------------------------
//         Global functions
//------------------------------------------------------------------------------
extern void BOOT_AT45_EraseBoot();
extern void BOOT_NAND_EraseBoot();
extern int BOOT_AT45_CopyBin(const Tdesc *pTd, unsigned char nbTd);
extern int BOOT_AT26_CopyBin(const Tdesc *pTd, unsigned char nbTd);
extern int BOOT_NAND_CopyBin(const Tdesc *pTd, unsigned char nbTd);
extern int BOOT_NOR_CopyBin(const Tdesc *pTd, unsigned char nbTd);
extern int BOOT_SDcard_CopyFile(const Tdesc *pTd, unsigned char nbTd);
extern int Acme_SDcard_CopyFile(char *filename,unsigned long destination,unsigned long size);
extern void Acme_SDcard_Stop();

extern int BOOT_EEPROM_CopyBin(const Tdesc *pTd, unsigned char nbTd);

#endif /*MAIN_H_*/
