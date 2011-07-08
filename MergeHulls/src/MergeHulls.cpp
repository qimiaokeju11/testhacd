#include "MergeHulls.h"
#include "FloatMath.h"
#include "StanHull.h"
#include <stdlib.h>
#include <string.h>

#pragma warning(disable:4100)

static const physx::PxF32 EPSILON=0.0001f;

class QuickSortPointers
{
public:
	void qsort(void **b,physx::PxI32 num)
	{
		char *lo,*hi;
		char *mid;
		char *bottom, *top;
		physx::PxI32 size;
		char *lostk[30], *histk[30];
		physx::PxI32 stkptr;
		char **base = (char **)b;

		if (num < 2 ) return;

		stkptr = 0;

		lo = (char *)base;
		hi = (char *)base + sizeof(char **) * (num-1);

nextone:

		size = (physx::PxI32)((physx::PxI32)(hi - lo) / sizeof(char**) + 1);

		mid = lo + (size / 2) * sizeof(char **);
		swap((char **)mid,(char **)lo);
		bottom = lo;
		top = hi + sizeof(char **);

		for (;;)
		{
			do
			{
				bottom += sizeof(char **);
			} while (bottom <= hi && compare((void **)bottom,(void **)lo) <= 0);

			do
			{
				top -= sizeof(char **);
			} while (top > lo && compare((void **)top,(void **)lo) >= 0);

			if (top < bottom) break;

			swap((char **)bottom,(char **)top);

		}

		swap((char **)lo,(char **)top);

		if ( top - 1 - lo >= hi - bottom )
		{
			if (lo + sizeof(char **) < top)
			{
				lostk[stkptr] = lo;
				histk[stkptr] = top - sizeof(char **);
				stkptr++;
			}
			if (bottom < hi)
			{
				lo = bottom;
				goto nextone;
			}
		}
		else
		{
			if ( bottom < hi )
			{
				lostk[stkptr] = bottom;
				histk[stkptr] = hi;
				stkptr++;
			}
			if (lo + sizeof(char **) < top)
			{
				hi = top - sizeof(char **);
				goto nextone; 					/* do small recursion */
			}
		}

		stkptr--;

		if (stkptr >= 0)
		{
			lo = lostk[stkptr];
			hi = histk[stkptr];
			goto nextone;
		}
		return;
	}




protected:
	// -1 less, 0 equal, +1 greater.
	virtual physx::PxI32 compare(void **p1,void **p2) = 0;
private:
	PX_INLINE void swap(char **a,char **b)
	{
		char *tmp;

		if ( a != b )
		{
			tmp = *a;
			*a++ = *b;
			*b++ = tmp;
		}
	}

};


class PxMergeConvexHulls : public MergeConvexHulls, public physx::UserAllocated, public QuickSortPointers
{
public:

	class ConvexResult : public Hull, public physx::UserAllocated
	{
	public:
		ConvexResult(physx::PxU32 hvcount,const physx::PxF32 *hvertices,physx::PxU32 htcount,const physx::PxU32 *hindices)
		{
			mVertexCount = hvcount;
			if ( mVertexCount )
			{
				mVertices = (physx::PxF32 *)PX_ALLOC(mVertexCount*sizeof(physx::PxF32)*3);
				memcpy((void *)mVertices, hvertices, sizeof(physx::PxF32)*3*mVertexCount );
			}
			else
			{
				mVertices = 0;
			}

			mTriangleCount = htcount;

			if ( mTriangleCount )
			{
				mIndices = (physx::PxU32 *)PX_ALLOC(sizeof(physx::PxU32)*mTriangleCount*3);
				memcpy((void *)mIndices,hindices, sizeof(physx::PxU32)*mTriangleCount*3 );
			}
			else
			{
				mIndices = 0;
			}

		}

		ConvexResult(const ConvexResult &r) // copy constructor, perform a deep copy of the data.
		{
			mVertexCount = r.mVertexCount;
			if ( mVertexCount )
			{
				mVertices = (physx::PxF32 *)PX_ALLOC(mVertexCount*sizeof(physx::PxF32)*3);
				memcpy((void *)mVertices, r.mVertices, sizeof(physx::PxF32)*3*mVertexCount );
			}
			else
			{
				mVertices = 0;
			}
			mTriangleCount = r.mTriangleCount;
			if ( mTriangleCount )
			{
				mIndices = (physx::PxU32 *)PX_ALLOC(sizeof(physx::PxU32)*mTriangleCount*3);
				memcpy((void *)mIndices, r.mIndices, sizeof(physx::PxU32)*mTriangleCount*3 );
			}
			else
			{
				mIndices = 0;
			}
		}

		~ConvexResult(void)
		{
			PX_FREE((void *)mVertices);
			PX_FREE((void *)mIndices);
		}

	};

	class CHull : public UserAllocated
	{
	public:
		CHull(const ConvexResult &result)
		{
			mResult = PX_NEW(ConvexResult)(result);
			mVolume = physx::fm_computeMeshVolume( result.mVertices, result.mTriangleCount, result.mIndices );

			mDiagonal = physx::fm_computeBestFitAABB( result.mVertexCount, result.mVertices, sizeof(physx::PxF32)*3, mMin, mMax );

			physx::PxF32 dx = mMax[0] - mMin[0];
			physx::PxF32 dy = mMax[1] - mMin[1];
			physx::PxF32 dz = mMax[2] - mMin[2];

			dx*=0.1f; // inflate 1/10th on each edge
			dy*=0.1f; // inflate 1/10th on each edge
			dz*=0.1f; // inflate 1/10th on each edge

			mMin[0]-=dx;
			mMin[1]-=dy;
			mMin[2]-=dz;

			mMax[0]+=dx;
			mMax[1]+=dy;
			mMax[2]+=dz;


		}

		~CHull(void)
		{
			delete mResult;
		}

		bool overlap(const CHull &h) const
		{
			return physx::fm_intersectAABB(mMin,mMax, h.mMin, h.mMax );
		}

		physx::PxF32		mMin[3];
		physx::PxF32		mMax[3];
		physx::PxF32		mVolume;
		physx::PxF32		mDiagonal; // long edge..
		ConvexResult		*mResult;
	};

	// Usage: std::sort( list.begin(), list.end(), StringSortRef() );
	class CHullSort
	{
	public:

		bool operator()(const CHull *a,const CHull *b) const
		{
			return a->mVolume < b->mVolume;
		}
	};


	typedef STDNAME::vector< CHull * > CHullVector;
	typedef STDNAME::vector<ConvexResult *> ConvexResultVector;






	PxMergeConvexHulls(void)
	{

	}

	virtual ~PxMergeConvexHulls(void)
	{
		for (physx::PxU32 i=0; i<mChulls.size(); i++)
		{
			CHull *h = mChulls[i];
			delete h;
		}
	}


	virtual void			addConvexHull(const Hull &h)
	{
		ConvexResult result(h.mVertexCount,h.mVertices,h.mTriangleCount,h.mIndices);
		CHull *hull = PX_NEW(CHull)(result);
		mChulls.push_back(hull);
	}

	virtual	physx::PxU32	performMerge(physx::PxF32 mergePercentage,physx::PxU32 maxHullVertices) 
	{
		while ( combineHulls(mergePercentage,maxHullVertices ));
		return (physx::PxU32)mChulls.size();
	}

	virtual	const Hull		*getMergedHull(physx::PxU32 index)
	{
		Hull *ret = NULL;
		if ( index < mChulls.size() )
		{
			CHull *cr = mChulls[index];
			ret = static_cast< Hull *>(cr->mResult);
		}
		return ret;
	}

	virtual	void			release(void)
	{
		delete this;
	}

private:

	virtual physx::PxI32 compare(void **p1,void **p2)
	{
		CHull **cp1 = (CHull **)p1;
		CHull **cp2 = (CHull **)p2;
		CHull *h1 = cp1[0];
		CHull *h2 = cp2[0];

		if ( h1->mVolume > h2->mVolume )
			return -1;
		else if ( h1->mVolume < h2->mVolume )
			return 1;
		return 0;
	}



	void sortChulls(CHullVector & hulls)
	{
		CHull **hptr = &hulls[0];
		qsort((void **)hptr,(physx::PxI32)hulls.size());
	}

	void getMesh(const ConvexResult &cr,physx::fm_VertexIndex *vc)
	{
		const physx::PxU32 *src = cr.mIndices;

		for (physx::PxU32 i=0; i<cr.mTriangleCount; i++)
		{
			size_t i1 = *src++;
			size_t i2 = *src++;
			size_t i3 = *src++;

			const physx::PxF32 *p1 = &cr.mVertices[i1*3];
			const physx::PxF32 *p2 = &cr.mVertices[i2*3];
			const physx::PxF32 *p3 = &cr.mVertices[i3*3];
			bool newPos;
			i1 = vc->getIndex(p1,newPos);
			i2 = vc->getIndex(p2,newPos);
			i3 = vc->getIndex(p3,newPos);
		}
	}


	CHull * canMerge(CHull *a,CHull *b,physx::PxF32 mergePercent,physx::PxU32 maxHullVertices)
	{
		if ( !a->overlap(*b) ) return 0; // if their AABB's (with a little slop) don't overlap, then return.

		if ( mergePercent < 0 ) return 0;

		assert( a->mVolume > 0 );
		assert( b->mVolume > 0 );

		CHull *ret = 0;

		// ok..we are going to combine both meshes into a single mesh
		// and then we are going to compute the concavity...

		physx::fm_VertexIndex *vc = physx::fm_createVertexIndex((physx::PxF32)EPSILON,false);

		getMesh( *a->mResult, vc);
		getMesh( *b->mResult, vc);

		physx::PxU32 vcount = vc->getVcount();
		const physx::PxF32 *vertices = vc->getVerticesFloat();

		physx::HullResult hresult;
		physx::HullLibrary hl;
		physx::HullDesc   desc;

		desc.SetHullFlag(physx::QF_TRIANGLES);

		desc.mVcount       = (physx::PxU32)vcount;
		desc.mVertices     = vertices;
		desc.mVertexStride = sizeof(physx::PxF32)*3;
		desc.mMaxVertices = maxHullVertices;

		physx::HullError hret = hl.CreateConvexHull(desc,hresult);

		if ( hret == physx::QE_OK )
		{
			physx::PxF32 combineVolume  = physx::fm_computeMeshVolume( hresult.mOutputVertices, hresult.mNumFaces, hresult.mIndices );
			physx::PxF32 sumVolume      = a->mVolume + b->mVolume;

			physx::PxF32 percent = (sumVolume*100) / combineVolume;

			if ( percent >= (100.0f-mergePercent)  )
			{
				ConvexResult cr(hresult.mNumOutputVertices, hresult.mOutputVertices, hresult.mNumFaces, hresult.mIndices);
				ret = PX_NEW(CHull)(cr);
			}
		}

		fm_releaseVertexIndex(vc);
		return ret;
	}



	bool combineHulls(physx::PxF32 mergePercentage,physx::PxU32 maxHullVertices)
	{

		bool combine = false;

		sortChulls(mChulls); // sort the convex hulls, largest volume to least...

		CHullVector output; // the output hulls...


		CHullVector::iterator i;

		for (i=mChulls.begin(); i!=mChulls.end() && !combine; ++i)
		{
			CHull *cr = (*i);

			CHullVector::iterator j;
			for (j=mChulls.begin(); j!=mChulls.end(); ++j)
			{
				CHull *match = (*j);

				if ( cr != match ) // don't try to merge a hull with itself, that be stoopid
				{

					CHull *merge = canMerge(cr,match,mergePercentage,maxHullVertices); // if we can merge these two....

					if ( merge )
					{
						output.push_back(merge);
						++i;
						while ( i != mChulls.end() )
						{
							CHull *cr = (*i);
							if ( cr != match )
							{
								output.push_back(cr);
							}
							i++;
						}

						delete cr;
						delete match;
						combine = true;
						break;
					}
				}
			}

			if ( combine )
			{
				break;
			}
			else
			{
				output.push_back(cr);
			}
		}

		if ( combine )
		{
			mChulls.clear();
			mChulls = output;
			output.clear();
		}


		return combine;
	}


	CHullVector				mChulls;
};

MergeConvexHulls *createMergeConvexHulls(void)
{
	PxMergeConvexHulls *ret = PX_NEW(PxMergeConvexHulls);
	return static_cast< MergeConvexHulls *>(ret);
}