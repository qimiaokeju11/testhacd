#ifndef PLATFORM_CONFIG_H

#define PLATFORM_CONFIG_H

// Modify this header file to make the HACD data types be compatible with your
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <new>

// This header file provides a brief compatibility layer between the PhysX and APEX SDK foundation header files.
// Modify this header file to your own data types and memory allocation routines and do a global find/replace if necessary

namespace hacd
{

	typedef signed __int64		HaI64;
	typedef signed int			HaI32;
	typedef signed short		HaI16;
	typedef signed char			HaI8;

	typedef unsigned __int64	HaU64;
	typedef unsigned int		HaU32;
	typedef unsigned short		HaU16;
	typedef unsigned char		HaU8;

	typedef float				HaF32;
	typedef double				HaF64;



	class PxEmpty;

#define HACD_SIGN_BITMASK		0x80000000

	// avoid unreferenced parameter warning (why not just disable it?)
	// PT: or why not just omit the parameter's name from the declaration????
#define HACD_FORCE_PARAMETER_REFERENCE(_P) (void)(_P);
#define HACD_UNUSED(_P) HACD_FORCE_PARAMETER_REFERENCE(_P)

#define HACD_ALLOC_ALIGNED(x,y) ::_aligned_malloc(x,y)
#define HACD_ALLOC(x) ::_aligned_malloc(x,16)
#define HACD_FREE(x) ::_aligned_free(x)

#define HACD_ASSERT(x) assert(x)
#define HACD_ALWAYS_ASSERT() assert(0)

#define HACD_INLINE inline
#define HACD_NOINLINE __declspec(noinline)
#define HACD_FORCE_INLINE __forceinline
#define HACD_PLACEMENT_NEW(p, T)  new(p) T

	class UserAllocated
	{
	public:
		HACD_INLINE void* operator new(size_t size,UserAllocated *t)
		{
			HACD_FORCE_PARAMETER_REFERENCE(size);
			return t;
		}

		HACD_INLINE void* operator new(size_t size,const char *className,const char* fileName, int lineno,size_t classSize)
		{
			HACD_FORCE_PARAMETER_REFERENCE(className);
			HACD_FORCE_PARAMETER_REFERENCE(fileName);
			HACD_FORCE_PARAMETER_REFERENCE(lineno);
			HACD_FORCE_PARAMETER_REFERENCE(classSize);
			return HACD_ALLOC(size);
		}

		inline void* operator new[](size_t size,const char *className,const char* fileName, int lineno,size_t classSize)
		{
			HACD_FORCE_PARAMETER_REFERENCE(className);
			HACD_FORCE_PARAMETER_REFERENCE(fileName);
			HACD_FORCE_PARAMETER_REFERENCE(lineno);
			HACD_FORCE_PARAMETER_REFERENCE(classSize);
			return HACD_ALLOC(size);
		}

		inline void  operator delete(void* p,UserAllocated *t)
		{
			HACD_FORCE_PARAMETER_REFERENCE(p);
			HACD_FORCE_PARAMETER_REFERENCE(t);
			HACD_ALWAYS_ASSERT(); // should never be executed
		}

		inline void  operator delete(void* p)
		{
			HACD_FREE(p);
		}

		inline void  operator delete[](void* p)
		{
			HACD_FREE(p);
		}

		inline void  operator delete(void *p,const char *className,const char* fileName, int line,size_t classSize)
		{
			HACD_FORCE_PARAMETER_REFERENCE(className);
			HACD_FORCE_PARAMETER_REFERENCE(fileName);
			HACD_FORCE_PARAMETER_REFERENCE(line);
			HACD_FORCE_PARAMETER_REFERENCE(classSize);
			HACD_FREE(p);
		}

		inline void  operator delete[](void *p,const char *className,const char* fileName, int line,size_t classSize)
		{
			HACD_FORCE_PARAMETER_REFERENCE(className);
			HACD_FORCE_PARAMETER_REFERENCE(fileName);
			HACD_FORCE_PARAMETER_REFERENCE(line);
			HACD_FORCE_PARAMETER_REFERENCE(classSize);
			HACD_FREE(p);
		}

	};

#define HACD_NEW(T) new(#T,__FILE__,__LINE__,sizeof(T)) T

}; // end HACD namespace

#define USE_STL 0 // set to 1 to use the standard template library for all code; if off it uses high performance custom containers which trap all memory allocations.
#define UANS hacd	// the user allocator namespace
#define USE_CONSTRAINT_BUILDER 1

#if USE_STL

#include <map>
#include <set>
#include <vector>
#define STDNAME std

#else

#include "PxVector.h"
#include "PxMapSet.h"

#define STDNAME hacd

#endif



#endif
