#include "MergeHulls.h"
#include "FloatMath.h"
#include "ConvexHull.h"

#include <string.h>

#pragma warning(disable:4100 4189)

using namespace hacd;

namespace HACD
{

static const HaF32 EPSILON=0.0001f;

	class ConvexResult
	{
	public:
		ConvexResult(void)
		{
			mHullVcount = 0;
			mHullVertices = 0;
			mHullTcount = 0;
			mHullIndices = 0;
		}

		// the convex hull.result
		hacd::HaU32		   	mHullVcount;			// Number of vertices in this convex hull.
		hacd::HaF32 			*mHullVertices;			// The array of vertices (x,y,z)(x,y,z)...
		hacd::HaU32       	mHullTcount;			// The number of triangles int he convex hull
		hacd::HaU32			*mHullIndices;			// The triangle indices (0,1,2)(3,4,5)...
		hacd::HaF32           mHullVolume;		    // the volume of the convex hull.

	};

	class MyConvexResult : public ConvexResult, public hacd::UserAllocated
	{
	public:
		MyConvexResult(hacd::HaU32 hvcount,const hacd::HaF32 *hvertices,hacd::HaU32 htcount,const hacd::HaU32 *hindices)
		{
			mHullVcount = hvcount;
			if ( mHullVcount )
			{
				mHullVertices = (hacd::HaF32 *)HACD_ALLOC(mHullVcount*sizeof(hacd::HaF32)*3);
				memcpy(mHullVertices, hvertices, sizeof(hacd::HaF32)*3*mHullVcount );
			}
			else
			{
				mHullVertices = 0;
			}

			mHullTcount = htcount;

			if ( mHullTcount )
			{
				mHullIndices = (hacd::HaU32 *)HACD_ALLOC(sizeof(hacd::HaU32)*mHullTcount*3);
				memcpy(mHullIndices,hindices, sizeof(hacd::HaU32)*mHullTcount*3 );
			}
			else
			{
				mHullIndices = 0;
			}

		}

		MyConvexResult(const MyConvexResult &r) // copy constructor, perform a deep copy of the data.
		{
			mHullVcount = r.mHullVcount;
			if ( mHullVcount )
			{
				mHullVertices = (hacd::HaF32 *)HACD_ALLOC(mHullVcount*sizeof(hacd::HaF32)*3);
				memcpy(mHullVertices, r.mHullVertices, sizeof(hacd::HaF32)*3*mHullVcount );
			}
			else
			{
				mHullVertices = 0;
			}
			mHullTcount = r.mHullTcount;
			if ( mHullTcount )
			{
				mHullIndices = (hacd::HaU32 *)HACD_ALLOC(sizeof(hacd::HaU32)*mHullTcount*3);
				memcpy(mHullIndices, r.mHullIndices, sizeof(hacd::HaU32)*mHullTcount*3 );
			}
			else
			{
				mHullIndices = 0;
			}
		}

		~MyConvexResult(void)
		{
			HACD_FREE(mHullVertices);
			HACD_FREE(mHullIndices);
		}





	};


	class QuickSortPointers
	{
	public:
		void qsort(void **base,HaI32 num); // perform the qsort.
	protected:
		// -1 less, 0 equal, +1 greater.
		virtual HaI32 compare(void **p1,void **p2) = 0;
	private:
		void inline swap(char **a,char **b);
	};

	void QuickSortPointers::swap(char **a,char **b)
	{
		char *tmp;

		if ( a != b )
		{
			tmp = *a;
			*a++ = *b;
			*b++ = tmp;
		}
	}


	void QuickSortPointers::qsort(void **b,HaI32 num)
	{
		char *lo,*hi;
		char *mid;
		char *bottom, *top;
		HaI32 size;
		char *lostk[30], *histk[30];
		HaI32 stkptr;
		char **base = (char **)b;

		if (num < 2 ) return;

		stkptr = 0;

		lo = (char *)base;
		hi = (char *)base + sizeof(char **) * (num-1);

nextone:

		size = (HaI32)(hi - lo) / sizeof(char**) + 1;

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


	class CHull : public UserAllocated
	{
	public:
		CHull(const MyConvexResult &result)
		{
			mResult = HACD_NEW(MyConvexResult)(result);
			mVolume = fm_computeMeshVolume( result.mHullVertices, result.mHullTcount, result.mHullIndices );

			mDiagonal = fm_computeBestFitAABB( result.mHullVcount, result.mHullVertices, sizeof(hacd::HaF32)*3, mMin, mMax );

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
			return fm_intersectAABB(mMin,mMax, h.mMin, h.mMax );
		}

		hacd::HaF32          mMin[3];
		hacd::HaF32          mMax[3];
		hacd::HaF32          mVolume;
		hacd::HaF32          mDiagonal; // long edge..
		MyConvexResult  *mResult;
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
typedef STDNAME::vector<MyConvexResult *> ConvexResultVector;

class MyMergeHullsInterface : public MergeHullsInterface, public hacd::UserAllocated, public QuickSortPointers
{
public:
	MyMergeHullsInterface(void)
	{

	}

	virtual ~MyMergeHullsInterface(void)
	{

	}

	// Merge these input hulls.
	virtual hacd::HaU32 mergeHulls(const MergeHullVector &inputHulls,
		MergeHullVector &outputHulls,
		hacd::HaF32	mergePercentage,
		hacd::HaF32	mergeTotalVolumePercentage)
	{
		hacd::HaU32 ret = 0;

		for (MergeHullVector::const_iterator i=inputHulls.begin(); i!=inputHulls.end(); ++i)
		{
			const MergeHull &h = (*i);
			MyConvexResult result(h.mVertexCount,h.mVertices,h.mTriangleCount,h.mIndices);
			ConvexDecompResult(result);
		}

		while ( combineHulls() ); // keep combinging hulls until I can't combine any more...

		return ret;
	}

	virtual void ConvexDecompResult(MyConvexResult &result)
	{
		CHull *ch = HACD_NEW(CHull)(result);
		if ( ch->mVolume > 0.00001f )
		{
			mChulls.push_back(ch);
		}
		else
		{
			delete ch;
		}
	}


	virtual void release(void) 
	{
		delete this;
	}

	void getMesh(const MyConvexResult &cr,fm_VertexIndex *vc)
	{
		hacd::HaU32 *src = cr.mHullIndices;

		for (hacd::HaU32 i=0; i<cr.mHullTcount; i++)
		{
			size_t i1 = *src++;
			size_t i2 = *src++;
			size_t i3 = *src++;

			const hacd::HaF32 *p1 = &cr.mHullVertices[i1*3];
			const hacd::HaF32 *p2 = &cr.mHullVertices[i2*3];
			const hacd::HaF32 *p3 = &cr.mHullVertices[i3*3];
			bool newPos;
			i1 = vc->getIndex(p1,newPos);
			i2 = vc->getIndex(p2,newPos);
			i3 = vc->getIndex(p3,newPos);
		}
	}



	CHull * canMerge(CHull *a,CHull *b)
	{
		if ( !a->overlap(*b) ) return 0; // if their AABB's (with a little slop) don't overlap, then return.

		if ( mMergePercent < 0 ) return 0;

		assert( a->mVolume > 0 );
		assert( b->mVolume > 0 );

		CHull *ret = 0;

		// ok..we are going to combine both meshes into a single mesh
		// and then we are going to compute the concavity...

		fm_VertexIndex *vc = fm_createVertexIndex((hacd::HaF32)EPSILON,false);

		getMesh( *a->mResult, vc);
		getMesh( *b->mResult, vc);

		size_t vcount = vc->getVcount();
		const hacd::HaF32 *vertices = vc->getVerticesFloat();

		HullResult hresult;
		HullLibrary hl;
		HullDesc   desc;
		desc.mVcount       = (hacd::HaU32)vcount;
		desc.mVertices     = vertices;
		desc.mVertexStride = sizeof(hacd::HaF32)*3;
		HullError hret = hl.CreateConvexHull(desc,hresult);
		if ( hret == QE_OK )
		{
			hacd::HaF32 combineVolume  = fm_computeMeshVolume( hresult.mOutputVertices, hresult.mNumTriangles, hresult.mIndices );
			hacd::HaF32 sumVolume      = a->mVolume + b->mVolume;

			hacd::HaF32 percent = (sumVolume*100) / combineVolume;

			if ( percent >= (100.0f-mMergePercent)  )
			{
				MyConvexResult cr(hresult.mNumOutputVertices, hresult.mOutputVertices, hresult.mNumTriangles, hresult.mIndices);
				ret = HACD_NEW(CHull)(cr);
			}
		}
		fm_releaseVertexIndex(vc);
		return ret;
	}

	void sortChulls(CHullVector & hulls)
	{
		CHull **hptr = &hulls[0];
		QuickSortPointers::qsort((void **)hptr,hulls.size());
	}

	bool combineHulls(void)
	{

		bool combine = false;

		sortChulls(mChulls); // sort the convex hulls, largest volume to least...

		CHullVector output; // the output hulls...


		CHullVector::iterator i;

		for (i=mChulls.begin(); i!=mChulls.end() && !combine; ++i)
		{
			CHull *cr = (*i);

			CHullVector::iterator j = i + 1;
			for (; j!=mChulls.end(); ++j)
			{
				CHull *match = (*j);

				if ( cr != match ) // don't try to merge a hull with itself, that be stoopid
				{

					CHull *merge = canMerge(cr,match); // if we can merge these two....
					if ( !merge )
					{
						merge = canMerge(match,cr);
					}

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


	virtual HaI32 compare(void **p1,void **p2)
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



private:
	HaF32				mMergePercent;
	CHullVector			mChulls;
	ConvexResultVector	mResults;

};

MergeHullsInterface * createMergeHullsInterface(void)
{
	MyMergeHullsInterface *m = HACD_NEW(MyMergeHullsInterface);
	return static_cast< MergeHullsInterface *>(m);
}


};