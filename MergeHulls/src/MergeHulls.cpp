#include "MergeHulls.h"
#include "FloatMath.h"
#include "StanHull.h"
#include <stdlib.h>
#include <string.h>

#pragma warning(disable:4100)

static const hacd::HaF32 EPSILON=0.0001f;

class QuickSortPointers
{
public:
	void qsort(void **b,hacd::HaI32 num)
	{
		char *lo,*hi;
		char *mid;
		char *bottom, *top;
		hacd::HaI32 size;
		char *lostk[30], *histk[30];
		hacd::HaI32 stkptr;
		char **base = (char **)b;

		if (num < 2 ) return;

		stkptr = 0;

		lo = (char *)base;
		hi = (char *)base + sizeof(char **) * (num-1);

nextone:

		size = (hacd::HaI32)((hacd::HaI32)(hi - lo) / sizeof(char**) + 1);

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
	virtual hacd::HaI32 compare(void **p1,void **p2) = 0;
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


class PxMergeConvexHulls : public MergeConvexHulls, public hacd::UserAllocated, public QuickSortPointers
{
public:

	class ConvexResult : public Hull, public hacd::UserAllocated
	{
	public:
		ConvexResult(hacd::HaU32 hvcount,const hacd::HaF32 *hvertices,hacd::HaU32 htcount,const hacd::HaU32 *hindices)
		{
			mVertexCount = hvcount;
			if ( mVertexCount )
			{
				mVertices = (hacd::HaF32 *)PX_ALLOC(mVertexCount*sizeof(hacd::HaF32)*3);
				memcpy((void *)mVertices, hvertices, sizeof(hacd::HaF32)*3*mVertexCount );
			}
			else
			{
				mVertices = 0;
			}

			mTriangleCount = htcount;

			if ( mTriangleCount )
			{
				mIndices = (hacd::HaU32 *)PX_ALLOC(sizeof(hacd::HaU32)*mTriangleCount*3);
				memcpy((void *)mIndices,hindices, sizeof(hacd::HaU32)*mTriangleCount*3 );
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
				mVertices = (hacd::HaF32 *)PX_ALLOC(mVertexCount*sizeof(hacd::HaF32)*3);
				memcpy((void *)mVertices, r.mVertices, sizeof(hacd::HaF32)*3*mVertexCount );
			}
			else
			{
				mVertices = 0;
			}
			mTriangleCount = r.mTriangleCount;
			if ( mTriangleCount )
			{
				mIndices = (hacd::HaU32 *)PX_ALLOC(sizeof(hacd::HaU32)*mTriangleCount*3);
				memcpy((void *)mIndices, r.mIndices, sizeof(hacd::HaU32)*mTriangleCount*3 );
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
			mVolume = hacd::fm_computeMeshVolume( result.mVertices, result.mTriangleCount, result.mIndices );

			mDiagonal = hacd::fm_computeBestFitAABB( result.mVertexCount, result.mVertices, sizeof(hacd::HaF32)*3, mMin, mMax );

			hacd::HaF32 dx = mMax[0] - mMin[0];
			hacd::HaF32 dy = mMax[1] - mMin[1];
			hacd::HaF32 dz = mMax[2] - mMin[2];

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
			return hacd::fm_intersectAABB(mMin,mMax, h.mMin, h.mMax );
		}

		hacd::HaF32		mMin[3];
		hacd::HaF32		mMax[3];
		hacd::HaF32		mVolume;
		hacd::HaF32		mDiagonal; // long edge..
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
		for (hacd::HaU32 i=0; i<mChulls.size(); i++)
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

	virtual	hacd::HaU32	performMerge(hacd::HaF32 mergePercentage,hacd::HaU32 maxHullVertices) 
	{
		while ( combineHulls(mergePercentage,maxHullVertices ));
		return (hacd::HaU32)mChulls.size();
	}

	virtual	const Hull		*getMergedHull(hacd::HaU32 index)
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

	virtual hacd::HaI32 compare(void **p1,void **p2)
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
		qsort((void **)hptr,(hacd::HaI32)hulls.size());
	}

	void getMesh(const ConvexResult &cr,hacd::fm_VertexIndex *vc)
	{
		const hacd::HaU32 *src = cr.mIndices;

		for (hacd::HaU32 i=0; i<cr.mTriangleCount; i++)
		{
			size_t i1 = *src++;
			size_t i2 = *src++;
			size_t i3 = *src++;

			const hacd::HaF32 *p1 = &cr.mVertices[i1*3];
			const hacd::HaF32 *p2 = &cr.mVertices[i2*3];
			const hacd::HaF32 *p3 = &cr.mVertices[i3*3];
			bool newPos;
			i1 = vc->getIndex(p1,newPos);
			i2 = vc->getIndex(p2,newPos);
			i3 = vc->getIndex(p3,newPos);
		}
	}


	CHull * canMerge(CHull *a,CHull *b,hacd::HaF32 mergePercent,hacd::HaU32 maxHullVertices)
	{
		if ( !a->overlap(*b) ) return 0; // if their AABB's (with a little slop) don't overlap, then return.

		if ( mergePercent < 0 ) return 0;

		assert( a->mVolume > 0 );
		assert( b->mVolume > 0 );

		CHull *ret = 0;

		// ok..we are going to combine both meshes into a single mesh
		// and then we are going to compute the concavity...

		hacd::fm_VertexIndex *vc = hacd::fm_createVertexIndex((hacd::HaF32)EPSILON,false);

		getMesh( *a->mResult, vc);
		getMesh( *b->mResult, vc);

		hacd::HaU32 vcount = vc->getVcount();
		const hacd::HaF32 *vertices = vc->getVerticesFloat();

		hacd::HullResult hresult;
		hacd::HullLibrary hl;
		hacd::HullDesc   desc;

		desc.SetHullFlag(hacd::QF_TRIANGLES);

		desc.mVcount       = (hacd::HaU32)vcount;
		desc.mVertices     = vertices;
		desc.mVertexStride = sizeof(hacd::HaF32)*3;
		desc.mMaxVertices = maxHullVertices;

		hacd::HullError hret = hl.CreateConvexHull(desc,hresult);

		if ( hret == hacd::QE_OK )
		{
			hacd::HaF32 combineVolume  = hacd::fm_computeMeshVolume( hresult.mOutputVertices, hresult.mNumFaces, hresult.mIndices );
			hacd::HaF32 sumVolume      = a->mVolume + b->mVolume;

			hacd::HaF32 percent = (sumVolume*100) / combineVolume;

			if ( percent >= (100.0f-mergePercent)  )
			{
				ConvexResult cr(hresult.mNumOutputVertices, hresult.mOutputVertices, hresult.mNumFaces, hresult.mIndices);
				ret = PX_NEW(CHull)(cr);
			}
		}

		fm_releaseVertexIndex(vc);
		return ret;
	}



	bool combineHulls(hacd::HaF32 mergePercentage,hacd::HaU32 maxHullVertices)
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