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

#define FILE_NAME ""
#define STR_DESCR ""
#define BIN_SIZE 0
#define DEST_ADDR 0


#ifndef BOOT_H_
#define BOOT_H_

//-----------------------------------------------------------------------------
//         Headers
//-----------------------------------------------------------------------------

#include "main.h"

//-----------------------------------------------------------------------------
//         Macros
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
//         Defines
//-----------------------------------------------------------------------------
/// MACH_TYPE this define is used by u-boot to launch Linux
#if defined(at91sam9260ek)
#define MACH_TYPE       0x44B       /* AT91SAM9260-EK */
#elif defined(at91sam9261ek)
#define MACH_TYPE       0x350       /* AT91SAM9261-EK */
#elif defined(at91sam9263ek)
#define MACH_TYPE       0x4B2       /* AT91SAM9263-EK */
#elif defined(at91sam9g20ek)
#define MACH_TYPE       0x658       /* AT91SAM9G20-EK */
#elif defined(at91sam9rlek)
#define MACH_TYPE       1326        /* AT91SAM9RL-EK */
#elif defined(at91sam9xeek)
#define MACH_TYPE       0x44B       /* AT91SAM9XE-EK same id as AT91SAM9260-EK*/
#elif defined(at91sam9m10ek)
#define MACH_TYPE       1830        /* AT91SAM9M10 */
#elif defined(at91cap9)
#define MACH_TYPE 0x05A2            /* AT91CAP9 */
#elif defined(at91sam9g45ek)
#define MACH_TYPE       2212        /* AT91SAM9G45-EK */
#elif defined(at91sam9g10ek)
#define MACH_TYPE       2159        /* AT91SAM9G10-EK */

#endif

/// BOOT_RECOVERY is enabled by default for the at91sam9269
/// If BOOT_RECOVERY option is enabled, then nvm first sector can be erased
/// by pushing a button after power up sequence. After the following power-up
/// sequence, the ROM code will execute until SAM-BA boot.
#if defined(at91sam9260ek)
#define BOOT_RECOVERY 1
#endif

//-----------------------------------------------------------------------------
//         Variables
//-----------------------------------------------------------------------------

#if defined(ORIGIN_sdcard)
// memory source address is not used for SDcard boot (only STR_FILE)
#define FROM_ADDR 0
#else
// file name is not used for non formated memory boot (only FROM_ADDR)
#define FILE_NAME ""
#endif

#if !defined(STR_DESCR)
#define STR_DESCR ""
#endif

#if defined(FROM_ADDR) &&  \
    defined(FILE_NAME)  &&  \
    defined(DEST_ADDR) &&  \
    defined(BIN_SIZE)  &&  \
    defined(STR_DESCR)    
    
const Tdesc tabDesc[] = {
    {FROM_ADDR, FILE_NAME, DEST_ADDR, BIN_SIZE, STR_DESCR}    
};

#else

    #if !defined(FROM_ADDR)
    #error FROM_ADDR not defined !
    #endif
    #if !defined(FILE_NAME)
    #error FILE_NAME not defined !
    #endif
    #if !defined(DEST_ADDR)
    #error DEST_ADDR not defined !
    #endif
    #if !defined(BIN_SIZE)
    #error BIN_SIZE not defined !
    #endif
    #if !defined(STR_DESCR) 
    #error STR_DESCR not defined !
    #endif

#endif /* Default arguments */

#endif /*BOOT_H_*/

