------------------------------------------------------------------------------- 

AcmeBoot is a tiny boot loader for the Acme Systems Linux SBC 
FOX Board G20 (http://foxg20.acmesystems.it) made by 
Acme Systems srl (http://www.acmesystems.it)

The AcmeBoot web page is available here:
http://foxg20.acmesystems.it/doku.php?id=dev:acmeboot

------------------------------------------------------------------------------- 

To compile AcmeBoot you have to get the CodeSourcery toolchain 
from this URL:
http://www.codesourcery.com/sgpp/lite/arm/portal/release642

Download the arm-2008q3-66-arm-none-eabi.bin 
file on your home directory and install it typing:

chmod +x arm-2008q3-66-arm-none-eabi.bin 
$ ./arm-2008q3-66-arm-none-eabi.bin

Follow the installation messages using the default values.

------------------------------------------------------------------------------- 

To launch the AcmeBoot compilation type:

cd AcmeBoot/AcmeBootProject
./doit.sh

It creates the ./bin/boot-NetusG20-sdcard2sdram.bin to use with
Pizzica In-System programming utility as illustrate on:
http://foxg20.acmesystems.it/doku.php?id=dev:pizzica

------------------------------------------------------------------------------- 

AcmeBoot is based on Atmel AT91Bootstrap v.3.0

THIS SOFTWARE IS PROVIDED BY ACME SYSTEMS "AS IS" IN THE SAME 
TERMS OF THE ORIGINAL ATMEL DISCLAIMER LISTED BELOW


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
