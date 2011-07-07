#include "HACD.h"

#include "hacdHACD.h"

#pragma warning(disable:4100 4996)


namespace HACD
{

class MyHACD_API : public HACD_API, public physx::UserAllocated, public HACD::CallBackFunction
{
public:
	MyHACD_API(void)
	{
		mHACD = NULL;
		mHull.mIndices = NULL;
		mHull.mVertices = NULL;
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
				mPoints.pushBack(p);
			}
			for (physx::PxU32 i=0; i<desc.mTriangleCount; i++)
			{
				Vec3<physx::PxI32> t;
				t.X() = desc.mIndices[i*3+0];
				t.Y() = desc.mIndices[i*3+1];
				t.Z() = desc.mIndices[i*3+2];
				mTriangles.pushBack(t);
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
		}

		return ret;
	}

	void releaseHull(void)
	{
		PX_FREE((void *)mHull.mIndices);
		PX_FREE((void *)mHull.mVertices);
		mHull.mIndices = NULL;
		mHull.mVertices = NULL;
	}

	virtual const Hull *getHull(physx::PxU32 index)
	{
		releaseHull();

		mHull.mVertexCount = (physx::PxU32)mHACD->GetNPointsCH(index);
		mHull.mTriangleCount = (physx::PxU32)mHACD->GetNTrianglesCH(index);
		mHull.mVertices = (physx::PxF32 *)PX_ALLOC(sizeof(physx::PxF32)*3*mHull.mVertexCount);
		mHull.mIndices = (physx::PxU32 *)PX_ALLOC(sizeof(physx::PxU32)*3*mHull.mTriangleCount);

		Vec3<physx::PxF64> * pointsCH = (Vec3<physx::PxF64> *)PX_ALLOC(sizeof(Vec3<physx::PxF64>)*mHull.mVertexCount);
		Vec3<physx::PxI32> * trianglesCH = (Vec3<physx::PxI32> *)PX_ALLOC(sizeof(Vec3<physx::PxI32>)*mHull.mTriangleCount);

		mHACD->GetCH(index, pointsCH, trianglesCH);
		physx::PxF32 *hullVertices = (physx::PxF32 *)mHull.mVertices;
		for (physx::PxU32 i=0; i<mHull.mVertexCount; i++)
		{
			hullVertices[i*3+0] = (physx::PxF32)pointsCH[i].X();
			hullVertices[i*3+1] = (physx::PxF32)pointsCH[i].Y();
			hullVertices[i*3+2] = (physx::PxF32)pointsCH[i].Z();
		}
		physx::PxU32 *hullIndices = (physx::PxU32 *)mHull.mIndices;
		for (physx::PxU32 i=0; i<mHull.mTriangleCount; i++)
		{
			hullIndices[i*3+0] = trianglesCH[i].X();
			hullIndices[i*3+1] = trianglesCH[i].Y();
			hullIndices[i*3+2] = trianglesCH[i].Z();
		}
		PX_FREE(pointsCH);
		PX_FREE(trianglesCH);
		return &mHull;
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
		releaseHull();
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


	Hull		mHull; // temporary result information..
	HACD		*mHACD;
	Vec3Vector	mPoints;
	TriVector	mTriangles;
	UserCallback	*mUserCallback;

};

HACD_API * createHACD_API(void)
{
	MyHACD_API *m = PX_NEW(MyHACD_API);
	return static_cast<HACD_API *>(m);
}


};



