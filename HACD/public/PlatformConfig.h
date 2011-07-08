#ifndef PLATFORM_CONFIG_H

#define PLATFORM_CONFIG_H

#include "PxSimpleTypes.h"

#define USE_STL 1 // set to 1 to use the standard template library for all code; if off it uses high performance custom containers which trap all memory allocations.

#if USE_STL
#include <map>
#include <set>
#include <vector>
#define STDNAME std

#else

#include "PxVector.h"
#include "PxMapSet.h"

#define STDNAME physx

#endif

#endif
