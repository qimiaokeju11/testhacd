/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef __dgCRC__
#define __dgCRC__

//#include "dgStdafx.h"
#include "dgTypes.h"

hacd::HaU32 dgApi dgCRC (const char* const string);
hacd::HaU32 dgApi dgCRC (const void* const buffer, hacd::HaI32 size, hacd::HaU32 crcAcc = 0);
const char* dgApi dgInverseCRC (hacd::HaU32 CRC);

hacd::HaU32 dgApi dgHash (const void* const string, hacd::HaI32 size);

#endif

