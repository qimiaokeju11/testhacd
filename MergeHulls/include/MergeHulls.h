#ifndef MERGE_HULLS_H

#define MERGE_HULLS_H

#include "PlatformConfig.h"

class MergeConvexHulls
{
public:
	class Hull
	{
		public:
			physx::PxU32	mVertexCount;
			physx::PxU32	mTriangleCount;
			const physx::PxF32	*mVertices;
			const physx::PxU32	*mIndices;
	};

	virtual void			addConvexHull(const Hull &h) = 0;

	virtual	physx::PxU32	performMerge(physx::PxF32 mergePercentage) = 0;
	virtual	const Hull		*getMergedHull(physx::PxU32 index) = 0;
	virtual	void			release(void) = 0;
protected:
	virtual ~MergeConvexHulls(void)
	{
	}
};

MergeConvexHulls *createMergeConvexHulls(void);

#endif
