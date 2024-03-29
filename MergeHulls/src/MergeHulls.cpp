#include "MergeHulls.h"
#include "ConvexHull.h"

#include <string.h>
#include <math.h>
#include <hash_map>

#pragma warning(disable:4100 4189 4996)

using namespace hacd;

namespace HACD
{

typedef stdext::hash_map< HaU32, HaF32 > TestedMap;

static HaF32 fm_computeBestFitAABB(HaU32 vcount,const HaF32 *points,HaU32 pstride,HaF32 *bmin,HaF32 *bmax) // returns the diagonal distance
{

	const HaU8 *source = (const HaU8 *) points;

	bmin[0] = points[0];
	bmin[1] = points[1];
	bmin[2] = points[2];

	bmax[0] = points[0];
	bmax[1] = points[1];
	bmax[2] = points[2];


	for (HaU32 i=1; i<vcount; i++)
	{
		source+=pstride;
		const HaF32 *p = (const HaF32 *) source;

		if ( p[0] < bmin[0] ) bmin[0] = p[0];
		if ( p[1] < bmin[1] ) bmin[1] = p[1];
		if ( p[2] < bmin[2] ) bmin[2] = p[2];

		if ( p[0] > bmax[0] ) bmax[0] = p[0];
		if ( p[1] > bmax[1] ) bmax[1] = p[1];
		if ( p[2] > bmax[2] ) bmax[2] = p[2];

	}

	HaF32 dx = bmax[0] - bmin[0];
	HaF32 dy = bmax[1] - bmin[1];
	HaF32 dz = bmax[2] - bmin[2];

	return (HaF32) ::sqrtf( dx*dx + dy*dy + dz*dz );

}




static bool fm_intersectAABB(const HaF32 *bmin1,const HaF32 *bmax1,const HaF32 *bmin2,const HaF32 *bmax2)
{
	if ((bmin1[0] > bmax2[0]) || (bmin2[0] > bmax1[0])) return false;
	if ((bmin1[1] > bmax2[1]) || (bmin2[1] > bmax1[1])) return false;
	if ((bmin1[2] > bmax2[2]) || (bmin2[2] > bmax1[2])) return false;
	return true;
}


static HACD_INLINE HaF32 det(const HaF32 *p1,const HaF32 *p2,const HaF32 *p3)
{
	return  p1[0]*p2[1]*p3[2] + p2[0]*p3[1]*p1[2] + p3[0]*p1[1]*p2[2] -p1[0]*p3[1]*p2[2] - p2[0]*p1[1]*p3[2] - p3[0]*p2[1]*p1[2];
}


static HaF32  fm_computeMeshVolume(const HaF32 *vertices,HaU32 tcount,const HaU32 *indices)
{
	HaF32 volume = 0;
	for (HaU32 i=0; i<tcount; i++,indices+=3)
	{
		const HaF32 *p1 = &vertices[ indices[0]*3 ];
		const HaF32 *p2 = &vertices[ indices[1]*3 ];
		const HaF32 *p3 = &vertices[ indices[2]*3 ];
		volume+=det(p1,p2,p3); // compute the volume of the tetrahedran relative to the origin.
	}

	volume*=(1.0f/6.0f);
	if ( volume < 0 )
		volume*=-1;
	return volume;
}



	class CHull : public UserAllocated
	{
	public:
		CHull(HaU32 vcount,const HaF32 *vertices,HaU32 tcount,const HaU32 *indices,HaU32 guid)
		{
			mGuid = guid;
			mVertexCount = vcount;
			mTriangleCount = tcount;
			mVertices = (HaF32 *)HACD_ALLOC(sizeof(HaF32)*3*vcount);
			memcpy(mVertices,vertices,sizeof(HaF32)*3*vcount);
			mIndices = (HaU32 *)HACD_ALLOC(sizeof(HaU32)*3*tcount);
			memcpy(mIndices,indices,sizeof(HaU32)*3*tcount);
			mVolume = fm_computeMeshVolume( mVertices, mTriangleCount, mIndices);
			mDiagonal = fm_computeBestFitAABB( mVertexCount, mVertices, sizeof(hacd::HaF32)*3, mMin, mMax );
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
			HACD_FREE(mVertices);
			HACD_FREE(mIndices);
		}

		bool overlap(const CHull &h) const
		{
			return fm_intersectAABB(mMin,mMax, h.mMin, h.mMax );
		}

		void getCenter(HaF32 *center) const
		{
			center[0] = (mMin[0] + mMax[0])*0.5f;
			center[1] = (mMin[1] + mMax[1])*0.5f;
			center[2] = (mMin[2] + mMax[2])*0.5f;
		}

		HaF32 distanceSquared(const CHull &h) const
		{
			HaF32 center1[3];
			HaF32 center2[3];
			getCenter(center1);
			h.getCenter(center2);
			HaF32 dx = center1[0] = center2[0];
			HaF32 dy = center1[1] = center2[1];
			HaF32 dz = center1[2] = center2[2];
			return dx*dx+dy*dy+dz*dz;
		}

		HaU32			mGuid;
		hacd::HaF32		mMin[3];
		hacd::HaF32		mMax[3];
		hacd::HaF32		mVolume;
		hacd::HaF32		mDiagonal; // long edge..
		HaU32			mVertexCount;
		HaU32			mTriangleCount;
		HaF32			*mVertices;
		HaU32			*mIndices;
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

class MyMergeHullsInterface : public MergeHullsInterface, public hacd::UserAllocated
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
		hacd::HaU32 mergeHullCount)
	{
		mGuid = 0;
		mHasBeenTested.clear();
		HaU32 maxMergeCount=0;
		if ( maxMergeCount < inputHulls.size() )
		{
			maxMergeCount = inputHulls.size() - mergeHullCount;
		}

		mTotalVolume = 0;
		for (HaU32 i=0; i<inputHulls.size(); i++)
		{
			const MergeHull &h = inputHulls[i];
			CHull *ch = HACD_NEW(CHull)(h.mVertexCount,h.mVertices,h.mTriangleCount,h.mIndices,mGuid++);
			mChulls.push_back(ch);
			mTotalVolume+=ch->mVolume;
		}

		for (HaU32 i=0; i<maxMergeCount; i++)
		{
			bool combined = combineHulls(); // mege smallest hulls first, up to the max merge count.
			if ( !combined ) break;
		}

		// return results..
		for (HaU32 i=0; i<mChulls.size(); i++)
		{
			CHull *ch = mChulls[i];
			MergeHull mh;
			mh.mVertexCount = ch->mVertexCount;
			mh.mTriangleCount = ch->mTriangleCount;
			mh.mIndices = ch->mIndices;
			mh.mVertices = ch->mVertices;
			outputHulls.push_back(mh);
		}

		return outputHulls.size();
	}

	virtual void ConvexDecompResult(hacd::HaU32 hvcount,const hacd::HaF32 *hvertices,hacd::HaU32 htcount,const hacd::HaU32 *hindices)
	{
		CHull *ch = HACD_NEW(CHull)(hvcount,hvertices,htcount,hindices,mGuid++);
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

	HaF32 canMerge(CHull *a,CHull *b)
	{
		if ( !a->overlap(*b) ) return 0; // if their AABB's (with a little slop) don't overlap, then return.

		// ok..we are going to combine both meshes into a single mesh
		// and then we are going to compute the concavity...
		HaF32 ret = 0;

		HaU32 combinedVertexCount = a->mVertexCount + b->mVertexCount;
		HaF32 *combinedVertices = (HaF32 *)HACD_ALLOC(combinedVertexCount*sizeof(HaF32)*3);
		HaF32 *dest = combinedVertices;
		memcpy(dest,a->mVertices, sizeof(HaF32)*3*a->mVertexCount);
		dest+=a->mVertexCount*3;
		memcpy(dest,b->mVertices,sizeof(HaF32)*3*b->mVertexCount);

		HullResult hresult;
		HullLibrary hl;
		HullDesc   desc;
		desc.mVcount       = combinedVertexCount;
		desc.mVertices     = combinedVertices;
		desc.mVertexStride = sizeof(hacd::HaF32)*3;
		HullError hret = hl.CreateConvexHull(desc,hresult);
		HACD_ASSERT( hret == QE_OK );
		if ( hret == QE_OK )
		{
			ret  = fm_computeMeshVolume( hresult.mOutputVertices, hresult.mNumTriangles, hresult.mIndices );
		}
		HACD_FREE(combinedVertices);
		hl.ReleaseResult(hresult);
		return ret;
	}


	CHull * doMerge(CHull *a,CHull *b)
	{
		CHull *ret = 0;
		HaU32 combinedVertexCount = a->mVertexCount + b->mVertexCount;
		HaF32 *combinedVertices = (HaF32 *)HACD_ALLOC(combinedVertexCount*sizeof(HaF32)*3);
		HaF32 *dest = combinedVertices;
		memcpy(dest,a->mVertices, sizeof(HaF32)*3*a->mVertexCount);
		dest+=a->mVertexCount*3;
		memcpy(dest,b->mVertices,sizeof(HaF32)*3*b->mVertexCount);
		HullResult hresult;
		HullLibrary hl;
		HullDesc   desc;
		desc.mVcount       = combinedVertexCount;
		desc.mVertices     = combinedVertices;
		desc.mVertexStride = sizeof(hacd::HaF32)*3;
		HullError hret = hl.CreateConvexHull(desc,hresult);
		HACD_ASSERT( hret == QE_OK );
		if ( hret == QE_OK )
		{
			ret = HACD_NEW(CHull)(hresult.mNumOutputVertices, hresult.mOutputVertices, hresult.mNumTriangles, hresult.mIndices,mGuid++);
		}
		HACD_FREE(combinedVertices);
		hl.ReleaseResult(hresult);
		return ret;
	}

	bool combineHulls(void)
	{
		bool combine = false;
		// each new convex hull is given a unique guid.
		// A hash map is used to make sure that no hulls are tested twice.
		CHullVector output;
		HaU32 count = mChulls.size();

		CHull *mergeA = NULL;
		CHull *mergeB = NULL;

		{
			HaF32 bestVolume = mTotalVolume;
			for (HaU32 i=0; i<count; i++)
			{
				CHull *cr = mChulls[i];
				for (HaU32 j=i+1; j<count; j++)
				{
					CHull *match = mChulls[j];
					HaU32 hashIndex;
					if ( match->mGuid < cr->mGuid )
					{
						hashIndex = (match->mGuid << 16) | cr->mGuid;
					}
					else
					{
						hashIndex = (cr->mGuid << 16 ) | match->mGuid;
					}
					HaF32 combinedVolume;
					TestedMap::iterator found = mHasBeenTested.find(hashIndex);
					if ( found == mHasBeenTested.end() )
					{
						combinedVolume = canMerge(cr,match);
						mHasBeenTested[hashIndex] = combinedVolume;
					}
					else
					{
						combinedVolume = (*found).second;
					}
					if ( combinedVolume != 0 )
					{
						if ( combinedVolume < bestVolume )
						{
							bestVolume = combinedVolume;
							mergeA = cr;
							mergeB = match;
						}
					}
				}
			}
		}
		if ( mergeA )
		{
			CHull *merge = doMerge(mergeA,mergeB);
			if ( merge )
			{
				combine = true;
				output.push_back(merge);
				for (CHullVector::iterator j=mChulls.begin(); j!=mChulls.end(); ++j)
				{
					CHull *h = (*j);
					if ( h !=mergeA && h != mergeB )
					{
						output.push_back(h);
					}
				}
				delete mergeA;
				delete mergeB;
			}
			mChulls = output;
#if 0
			static HaU32 mergeCount=0;
			mergeCount++;
			char scratch[512];
			sprintf_s(scratch,512,"Merge%03d.obj", mergeCount );
			FILE *fph = fopen(scratch,"wb");
			HaU32 baseVertex = 1;
			for (HaU32 i=0; i<mChulls.size(); i++)
			{
				CHull *h = mChulls[i];
				for (HaU32 i=0; i<h->mVertexCount; i++)
				{
					const HaF32 *p = &h->mVertices[i*3];
					fprintf(fph,"v %0.9f %0.9f %0.9f\r\n", p[0], p[1], p[2] );
				}
				for (HaU32 i=0; i<h->mTriangleCount; i++)
				{
					HaU32 i1 = h->mIndices[i*3+0];
					HaU32 i2 = h->mIndices[i*3+1];
					HaU32 i3 = h->mIndices[i*3+2];
					fprintf(fph,"f %d %d %d\r\n", i1+baseVertex, i2+baseVertex, i3+baseVertex );
				}
				baseVertex+=h->mVertexCount;
			}
			fclose(fph);
#endif
		}

		return combine;
	}

private:
	TestedMap			mHasBeenTested;
	HaU32				mGuid;
	HaF32				mTotalVolume;
	CHullVector			mChulls;
};

MergeHullsInterface * createMergeHullsInterface(void)
{
	MyMergeHullsInterface *m = HACD_NEW(MyMergeHullsInterface);
	return static_cast< MergeHullsInterface *>(m);
}


};