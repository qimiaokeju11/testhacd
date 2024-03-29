#ifndef CONVEX_HULL_H

#define CONVEX_HULL_H

#include "PlatformConfigHACD.h"

namespace HACD
{

class HullResult
{
public:
	HullResult(void)
	{
		mNumOutputVertices = 0;
		mOutputVertices = 0;
		mNumTriangles = 0;
		mIndices = 0;
	}
	hacd::HaU32			mNumOutputVertices;         // number of vertices in the output hull
	hacd::HaF32			*mOutputVertices;            // array of vertices, 3 floats each x,y,z
	hacd::HaU32			mNumTriangles;                  // the number of faces produced
	hacd::HaU32			*mIndices;                   // pointer to indices.

// If triangles, then indices are array indexes into the vertex list.
// If polygons, indices are in the form (number of points in face) (p1, p2, p3, ..) etc..
};

class HullDesc
{
public:
	HullDesc(void)
	{
		mVcount         = 0;
		mVertices       = 0;
		mVertexStride   = sizeof(hacd::HaF32)*3;
		mNormalEpsilon  = 0.001f;
		mMaxVertices = 4096; // maximum number of points to be considered for a convex hull.
		mSkinWidth = 0.0f; // default is one centimeter
	};

	HullDesc(hacd::HaU32 vcount,
			 const hacd::HaF32 *vertices,
			 hacd::HaU32 stride)
	{
		mVcount         = vcount;
		mVertices       = vertices;
		mVertexStride   = stride;
		mNormalEpsilon  = 0.001f;
		mMaxVertices    = 4096;
		mSkinWidth = 0.01f; // default is one centimeter
	}

	hacd::HaU32			mVcount;          // number of vertices in the input point cloud
	const hacd::HaF32	*mVertices;        // the array of vertices.
	hacd::HaU32			mVertexStride;    // the stride of each vertex, in bytes.
	hacd::HaF32			mNormalEpsilon;   // the epsilon for removing duplicates.  This is a normalized value, if normalized bit is on.
	hacd::HaF32			mSkinWidth;
	hacd::HaU32			mMaxVertices;               // maximum number of vertices to be considered for the hull!
};

enum HullError
{
	QE_OK,            // success!
	QE_FAIL,           // failed.
	QE_NOT_READY,
};

// This class is used when converting a convex hull into a triangle mesh.
class ConvexHullVertex
{
public:
	hacd::HaF32         mPos[3];
	hacd::HaF32         mNormal[3];
	hacd::HaF32         mTexel[2];
};

// A virtual interface to receive the triangles from the convex hull.
class ConvexHullTriangleInterface
{
public:
	virtual void ConvexHullTriangle(const ConvexHullVertex &v1,const ConvexHullVertex &v2,const ConvexHullVertex &v3) = 0;
};


class HullLibrary
{
public:

	HullError CreateConvexHull(const HullDesc       &desc,           // describes the input request
								HullResult           &result);        // contains the resulst

	HullError ReleaseResult(HullResult &result); // release memory allocated for this result, we are done with it.

	HullError CreateTriangleMesh(HullResult &answer,ConvexHullTriangleInterface *iface);
private:
	hacd::HaF32 ComputeNormal(hacd::HaF32 *n,const hacd::HaF32 *A,const hacd::HaF32 *B,const hacd::HaF32 *C);
	void AddConvexTriangle(ConvexHullTriangleInterface *callback,const hacd::HaF32 *p1,const hacd::HaF32 *p2,const hacd::HaF32 *p3);

	void BringOutYourDead(const hacd::HaF32 *verts,hacd::HaU32 vcount, hacd::HaF32 *overts,hacd::HaU32 &ocount,hacd::HaU32 *indices,hacd::HaU32 indexcount);

	bool    NormalizeAndCleanupVertices(hacd::HaU32 svcount,
							const hacd::HaF32 *svertices,
							hacd::HaU32 stride,
							hacd::HaU32 &vcount,       // output number of vertices
							hacd::HaF32 *vertices,                 // location to store the results.
							hacd::HaF32  normalepsilon,
							hacd::HaF32 *scale,
							hacd::HaF32 *center,
							hacd::HaU32 maxVertices);
};

}; // end of namespace HACD

#endif
