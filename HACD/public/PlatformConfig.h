#ifndef PLATFORM_CONFIG_H

#define PLATFORM_CONFIG_H

#include "PxSimpleTypes.h"

#define USE_STL 0

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
