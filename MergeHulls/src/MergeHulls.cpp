#include "MergeHulls.h"
#include <stdlib.h>
#include <string.h>

#pragma warning(disable:4100)

class PxMergeConvexHulls : public MergeConvexHulls, public physx::UserAllocated
{
public:
	PxMergeConvexHulls(void)
	{

	}

	virtual ~PxMergeConvexHulls(void)
	{
		for (physx::PxU32 i=0; i<mInputHulls.size(); i++)
		{
			Hull &h = mInputHulls[i];
			PX_FREE((void *)h.mIndices);
			PX_FREE((void *)h.mVertices);
		}
		for (physx::PxU32 i=0; i<mOutputHulls.size(); i++)
		{
			Hull &h = mOutputHulls[i];
			PX_FREE((void *)h.mIndices);
			PX_FREE((void *)h.mVertices);
		}

	}


	virtual void			addConvexHull(const Hull &h)
	{
		Hull hull;
		hull.mVertexCount = h.mVertexCount;
		hull.mTriangleCount = h.mTriangleCount;
		hull.mIndices = (physx::PxU32 *)PX_ALLOC(sizeof(physx::PxU32)*h.mTriangleCount*3);
		hull.mVertices = (physx::PxF32 *)PX_ALLOC(sizeof(physx::PxF32)*h.mVertexCount*3);
		memcpy((void *)hull.mIndices,h.mIndices,sizeof(physx::PxU32)*h.mTriangleCount*3);
		memcpy((void *)hull.mVertices,h.mVertices,sizeof(physx::PxF32)*h.mVertexCount*3);
		mInputHulls.push_back(hull);
	}

	virtual	physx::PxU32	performMerge(physx::PxF32 mergePercentage) 
	{
		return (physx::PxU32)mOutputHulls.size();
	}

	virtual	const Hull		*getMergedHull(physx::PxU32 index)
	{
		Hull *ret = NULL;
		if ( index < mOutputHulls.size() )
		{
			ret = &mOutputHulls[index];
		}
		return ret;
	}

	virtual	void			release(void)
	{
		delete this;
	}

protected:
	STDNAME::vector< Hull > mInputHulls;
	STDNAME::vector< Hull > mOutputHulls;
};
