/*
 * dlpc350_flashDevice.h
 *
 * This module provides flash related definitions.
 *
 * Copyright (C) {2016} Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

#ifndef __FLASHDEVICE_H
#define __FLASHDEVICE_H

#include <string>

#define FLASH_FILE_VERSION_LONG_ID 2

typedef struct _FlashDevice
{
    std::string             Mfg;             // Company name.
    unsigned short          Mfg_ID;          // Manufacturer ID stored in part.
    unsigned long long int  LMfg_ID;         // Long Manufacturer ID stored in part.

    std::string             Dev;             // Part number from data sheet.
    unsigned short          Dev_ID;          // Device ID stored in part.
    unsigned long long int  LDev_ID;         // Long Device ID stored in part.

    unsigned int            Size_MBit;       // 4, 8, or 16 MBit.
    unsigned char           Type;            // A, B or C algothrim (0, 1, 2)
    unsigned int            numSectors;
    unsigned int            SectorArr[256];       // List of sector addresses.
}FlashDevice;

#endif  //__FLASHDEVICE_H
