#ifndef MERGE_HULLS_H

#define MERGE_HULLS_H

#include "PlatformConfig.h"

class MergeConvexHulls
{
public:
	class Hull
	{
		public:
			hacd::HaU32	mVertexCount;
			hacd::HaU32	mTriangleCount;
			const hacd::HaF32	*mVertices;
			const hacd::HaU32	*mIndices;
	};

	virtual void			addConvexHull(const Hull &h) = 0;

	virtual	hacd::HaU32	performMerge(hacd::HaF32 mergePercentage,hacd::HaU32 maxHullVertices) = 0;
	virtual	const Hull		*getMergedHull(hacd::HaU32 index) = 0;
	virtual	void			release(void) = 0;
protected:
	virtual ~MergeConvexHulls(void)
	{
	}
};

MergeConvexHulls *createMergeConvexHulls(void);

#endif
