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

#ifndef __dgRandom__
#define __dgRandom__

#include "dgTypes.h"

const hacd::HaF64 fRandom = (hacd::HaF64 (1.0) / hacd::HaF64 ((hacd::HaU32)(0xffffffff))); 

// return a random number between 0 and 0xffffffff;
hacd::HaU32 dgApi dgRandom();

inline hacd::HaF32 dgfRandom()
{
	return (hacd::HaF32) (dgRandom() * fRandom);
}


void dgApi dgRandomize (hacd::HaU32 Seed);

#endif

