#ifndef HACD_H

#define HACD_H

#include <PxSimpleTypes.h>
#include <stdlib.h>

namespace HACD
{


class HACD_API
{
public:
	class UserCallback
	{
	public:
		virtual bool hacdProgressUpdate(const char *message, physx::PxF32 progress, physx::PxF32 concavity, physx::PxU32 nVertices) = 0;
	};
	class Desc
	{
	public:
		Desc(void)
		{
			init();
		}

		UserCallback		*mCallback;
		physx::PxU32		mTriangleCount;
		physx::PxU32		mVertexCount;
		const physx::PxF32		*mVertices;
		const physx::PxU32		*mIndices;
		physx::PxU32		mMinHullCount;
		physx::PxF32		mConcavity;
		physx::PxU32		mMaxHullVertices;
		void init(void)
		{
			mCallback = NULL;
			mTriangleCount = 0;
			mVertexCount = 0;
			mVertices = NULL;
			mIndices = NULL;
			mMinHullCount = 10;
			mConcavity = 100;
			mMaxHullVertices = 64;
		}
	};
	class Hull
	{
	public:
		physx::PxU32		mTriangleCount;
		physx::PxU32		mVertexCount;
		const physx::PxF32		*mVertices;
		const physx::PxU32		*mIndices;
	};

	virtual physx::PxU32	performHACD(const Desc &desc) = 0;
	virtual const Hull *getHull(physx::PxU32 index) = 0;
	virtual void	releaseHACD(void) = 0; // release memory associated with the last HACD request


	virtual void release(void) = 0; // release the HACD_API interface

};

HACD_API * createHACD_API(void);

};

#endif
