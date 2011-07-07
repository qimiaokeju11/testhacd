#ifndef PX_SIMPLE_TYPES_H

#define PX_SIMPLE_TYPES_H

// This header file provides a brief compatibility layer between the PhysX and APEX SDK foundation header files.
// Modify this header file to your own data types and memory allocation routines and do a global find/replace if necessary

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <new>

namespace physx
{

    typedef signed __int64		PxI64;
    typedef signed int			PxI32;
    typedef signed short		PxI16;
    typedef signed char			PxI8;

    typedef unsigned __int64	PxU64;
    typedef unsigned int		PxU32;
    typedef unsigned short		PxU16;
    typedef unsigned char		PxU8;

    typedef float				PxF32;
    typedef double				PxF64;



class PxEmpty;

#define PX_SIGN_BITMASK		0x80000000

// avoid unreferenced parameter warning (why not just disable it?)
// PT: or why not just omit the parameter's name from the declaration????
#define PX_FORCE_PARAMETER_REFERENCE(_P) (void)(_P);
#define PX_UNUSED(_P) PX_FORCE_PARAMETER_REFERENCE(_P)


#define PX_ALLOC(x) ::malloc(x)
#define PX_FREE(x) ::free(x)
#define PX_REALLOC(x,y) ::realloc(x,y)

#define PX_ASSERT(x) assert(x)
#define PX_ALWAYS_ASSERT() assert(0)

#define PX_INLINE inline
#define PX_NOINLINE __declspec(noinline)
#define PX_FORCE_INLINE __forceinline
#define PX_PLACEMENT_NEW(p, T)  new(p) T

class UserAllocated
{
public:
	PX_INLINE void* operator new(size_t size,UserAllocated *t)
	{
		PX_FORCE_PARAMETER_REFERENCE(size);
		return t;
	}

	PX_INLINE void* operator new(size_t size,const char *className,const char* fileName, int lineno,size_t classSize)
	{
		PX_FORCE_PARAMETER_REFERENCE(className);
		PX_FORCE_PARAMETER_REFERENCE(fileName);
		PX_FORCE_PARAMETER_REFERENCE(lineno);
		PX_FORCE_PARAMETER_REFERENCE(classSize);
		return PX_ALLOC(size);
	}

	inline void* operator new[](size_t size,const char *className,const char* fileName, int lineno,size_t classSize)
	{
		PX_FORCE_PARAMETER_REFERENCE(className);
		PX_FORCE_PARAMETER_REFERENCE(fileName);
		PX_FORCE_PARAMETER_REFERENCE(lineno);
		PX_FORCE_PARAMETER_REFERENCE(classSize);
		return PX_ALLOC(size);
	}

	inline void  operator delete(void* p,UserAllocated *t)
	{
		PX_FORCE_PARAMETER_REFERENCE(p);
		PX_FORCE_PARAMETER_REFERENCE(t);
		PX_ALWAYS_ASSERT(); // should never be executed
	}

	inline void  operator delete(void* p)
	{
		PX_FREE(p);
	}

	inline void  operator delete[](void* p)
	{
		PX_FREE(p);
	}

	inline void  operator delete(void *p,const char *className,const char* fileName, int line,size_t classSize)
	{
		PX_FORCE_PARAMETER_REFERENCE(className);
		PX_FORCE_PARAMETER_REFERENCE(fileName);
		PX_FORCE_PARAMETER_REFERENCE(line);
		PX_FORCE_PARAMETER_REFERENCE(classSize);
		PX_FREE(p);
	}

	inline void  operator delete[](void *p,const char *className,const char* fileName, int line,size_t classSize)
	{
		PX_FORCE_PARAMETER_REFERENCE(className);
		PX_FORCE_PARAMETER_REFERENCE(fileName);
		PX_FORCE_PARAMETER_REFERENCE(line);
		PX_FORCE_PARAMETER_REFERENCE(classSize);
		PX_FREE(p);
	}

};

#define PX_NEW(T) new(#T,__FILE__,__LINE__,sizeof(T)) T

};


#endif
