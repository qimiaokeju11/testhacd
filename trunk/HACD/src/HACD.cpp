#include "HACD.h"
#include "hacdHACD.h"
#include "MergeHulls.h"
#include <stdlib.h>
#include <string.h>

#pragma warning(disable:4100 4996)


namespace HACD
{

class MyHACD_API : public HACD_API, public physx::UserAllocated, public HACD::CallBackFunction
{
public:
	MyHACD_API(void)
	{
		mHACD = NULL;
		mUserCallback = NULL;
	}
	virtual ~MyHACD_API(void)
	{
		releaseHACD();
	}

	virtual physx::PxU32	performHACD(const Desc &desc) 
	{
		physx::PxU32 ret = 0;
		releaseHACD();
		mUserCallback = desc.mCallback;
		if ( desc.mVertexCount )
		{
			mHACD = PX_NEW(HACD);
			// convert the input data points into doubles
			for (physx::PxU32 i=0; i<desc.mVertexCount; i++)
			{
				Vec3<physx::PxF64> p;
				p.X() = desc.mVertices[i*3+0];
				p.Y() = desc.mVertices[i*3+1];
				p.Z() = desc.mVertices[i*3+2];
				mPoints.push_back(p);
			}
			for (physx::PxU32 i=0; i<desc.mTriangleCount; i++)
			{
				Vec3<physx::PxI32> t;
				t.X() = desc.mIndices[i*3+0];
				t.Y() = desc.mIndices[i*3+1];
				t.Z() = desc.mIndices[i*3+2];
				mTriangles.push_back(t);
			}

			mHACD->SetPoints(&mPoints[0]);
			mHACD->SetNPoints(desc.mVertexCount);
			mHACD->SetTriangles(&mTriangles[0]);
			mHACD->SetNTriangles(desc.mTriangleCount);
			mHACD->SetCompacityWeight(0.1);
			mHACD->SetVolumeWeight(0);
			mHACD->SetNClusters(desc.mMinHullCount);
			mHACD->SetCallBack(this);
			mHACD->SetAddExtraDistPoints(false);
			mHACD->SetAddNeighboursDistPoints(false);
			mHACD->SetAddFacesPoints(true);
			mHACD->SetNVerticesPerCH(desc.mMaxHullVertices);		// max of 64 vertices per convex-hull
			mHACD->SetConcavity(desc.mConcavity);
			mHACD->Compute();

			ret = (physx::PxU32)mHACD->GetNClusters();
			for (physx::PxU32 i=0; i<ret; i++)
			{
				Hull h;
				getHull(i,h);
				mHulls.push_back(h);
			}

			if ( desc.mMergePercentage > 0 )
			{
				printf("Attempting to merge %d convex hulls with a merge threshold of %0.2f%%\r\n", ret, desc.mMergePercentage );
				MergeConvexHulls *m = createMergeConvexHulls();
				for (physx::PxU32 i=0; i<ret; i++)
				{
					Hull &h = mHulls[i];
					MergeConvexHulls::Hull hull;
					hull.mVertexCount = h.mVertexCount;
					hull.mTriangleCount = h.mTriangleCount;
					hull.mVertices = h.mVertices;
					hull.mIndices = h.mIndices;
					m->addConvexHull(hull);
				}
				physx::PxU32 mergeCount =  m->performMerge(desc.mMergePercentage,desc.mMaxHullVertices);
				if ( mergeCount && mergeCount != ret )
				{
					printf("Merged %d convex hulls, output is now only %d\r\n", ret-mergeCount, mergeCount );
					releaseHACD();
					for (physx::PxU32 i=0; i<ret; i++)
					{
						const MergeConvexHulls::Hull *h = m->getMergedHull(i);
						if( h )
						{
							Hull hull;
							hull.mVertexCount = h->mVertexCount;
							hull.mTriangleCount = h->mTriangleCount;
							hull.mVertices = (physx::PxF32 *)PX_ALLOC(sizeof(physx::PxF32)*3*hull.mVertexCount);
							hull.mIndices = (physx::PxU32 *)PX_ALLOC(sizeof(physx::PxU32)*3*hull.mTriangleCount);
							memcpy((void *)hull.mVertices,h->mVertices,sizeof(physx::PxF32)*3*hull.mVertexCount);
							memcpy((void *)hull.mIndices,h->mIndices,sizeof(physx::PxU32)*3*hull.mTriangleCount);
							mHulls.push_back(hull);
						}
					}
					ret = (physx::PxU32)mHulls.size();
				}
				m->release();
			}
		}

		return ret;
	}

	void releaseHull(Hull &h)
	{
		PX_FREE((void *)h.mIndices);
		PX_FREE((void *)h.mVertices);
		h.mIndices = NULL;
		h.mVertices = NULL;
	}

	virtual const Hull		*getHull(physx::PxU32 index) 
	{
		const Hull *ret = NULL;
		if ( index < mHulls.size() )
		{
			ret = &mHulls[index];
		}
		return ret;
	}

	virtual void getHull(physx::PxU32 index,Hull &h)
	{
		h.mVertexCount = (physx::PxU32)mHACD->GetNPointsCH(index);
		h.mTriangleCount = (physx::PxU32)mHACD->GetNTrianglesCH(index);
		h.mVertices = (physx::PxF32 *)PX_ALLOC(sizeof(physx::PxF32)*3*h.mVertexCount);
		h.mIndices = (physx::PxU32 *)PX_ALLOC(sizeof(physx::PxU32)*3*h.mTriangleCount);

		Vec3<physx::PxF64> * pointsCH = (Vec3<physx::PxF64> *)PX_ALLOC(sizeof(Vec3<physx::PxF64>)*h.mVertexCount);
		Vec3<physx::PxI32> * trianglesCH = (Vec3<physx::PxI32> *)PX_ALLOC(sizeof(Vec3<physx::PxI32>)*h.mTriangleCount);

		mHACD->GetCH(index, pointsCH, trianglesCH);
		physx::PxF32 *hullVertices = (physx::PxF32 *)h.mVertices;
		for (physx::PxU32 i=0; i<h.mVertexCount; i++)
		{
			hullVertices[i*3+0] = (physx::PxF32)pointsCH[i].X();
			hullVertices[i*3+1] = (physx::PxF32)pointsCH[i].Y();
			hullVertices[i*3+2] = (physx::PxF32)pointsCH[i].Z();
		}
		physx::PxU32 *hullIndices = (physx::PxU32 *)h.mIndices;
		for (physx::PxU32 i=0; i<h.mTriangleCount; i++)
		{
			hullIndices[i*3+0] = trianglesCH[i].X();
			hullIndices[i*3+1] = trianglesCH[i].Y();
			hullIndices[i*3+2] = trianglesCH[i].Z();
		}
		PX_FREE(pointsCH);
		PX_FREE(trianglesCH);
	}

	virtual void	releaseHACD(void) // release memory associated with the last HACD request
	{
		if ( mHACD )
		{
			delete mHACD;
			mHACD = NULL;
		}
		mPoints.clear();
		mTriangles.clear();
		for (physx::PxU32 i=0; i<mHulls.size(); i++)
		{
			releaseHull(mHulls[i]);
		}
		mHulls.clear();
		mUserCallback = NULL;
	}


	virtual void release(void) // release the HACD_API interface
	{
		delete this;
	}

	virtual bool progressUpdate(const char * msg, physx::PxF64 progress, physx::PxF64 concavity, physx::PxU32 nVertices)
	{
		bool ret = true;
		if ( mUserCallback )
		{
			ret = mUserCallback->hacdProgressUpdate(msg,(physx::PxF32)progress,(physx::PxF32)concavity,nVertices);
		}
		else
		{
			printf("%s", msg );
		}
		return ret;
	}


	HACD					*mHACD;
	Vec3Vector				mPoints;
	TriVector				mTriangles;
	UserCallback			*mUserCallback;
	STDNAME::vector< Hull >	mHulls;

};

HACD_API * createHACD_API(void)
{
	MyHACD_API *m = PX_NEW(MyHACD_API);
	return static_cast<HACD_API *>(m);
}


};



