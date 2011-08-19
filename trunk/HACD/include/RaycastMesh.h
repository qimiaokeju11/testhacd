#ifndef RAYCAST_MESH_H

#define RAYCAST_MESH_H

// This code snippet allows you to create an axis aligned bounding volume tree for a triangle mesh so that you can do
// high-speed raycasting.
//
// There are much better implementations of this available on the internet.  In particular I recommend that you use 
// OPCODE written by Pierre Terdiman.
// @see: http://www.codercorner.com/Opcode.htm
//
// OPCODE does a whole lot more than just raycasting, and is a rather significant amount of source code.
//
// I am providing this code snippet for the use case where you *only* want to do quick and dirty optimized raycasting.
// I have not done performance testing between this version and OPCODE; so I don't know how much slower it is.  However,
// anytime you switch to using a spatial data structure for raycasting, you increase your performance by orders and orders 
// of magnitude; so this implementation should work fine for simple tools and utilities.
//
// It also serves as a nice sample for people who are trying to learn the algorithm of how to implement AABB trees.
// AABB = Axis Aligned Bounding Volume trees.
//
// http://www.cgal.org/Manual/3.5/doc_html/cgal_manual/AABB_tree/Chapter_main.html
//
//
// This code snippet was written by John W. Ratcliff on August 18, 2011 and released under the MIT. license.
//
// mailto:jratcliffscarab@gmail.com
//
// The official source can be found at:  http://code.google.com/p/raycastmesh/
//
// 

#include "PlatformConfigHACD.h"

namespace hacd
{

class RaycastMesh
{
public:
	virtual bool raycast(const HaF64 *from,const HaF64 *to,HaF64 *hitLocation,HaF64 *hitNormal,HaF64 *hitDistance) = 0;
	virtual const HaF64 * getBoundMin(void) const = 0; // return the minimum bounding box
	virtual const HaF64 * getBoundMax(void) const = 0; // return the maximum bounding box.
	virtual void release(void) = 0;
protected:
	virtual ~RaycastMesh(void) { };
};


RaycastMesh * createRaycastMesh(HaU32 vcount,		// The number of vertices in the source triangle mesh
								const HaF64 *vertices,		// The array of vertex positions in the format x1,y1,z1..x2,y2,z2.. etc.
								HaU32 tcount,		// The number of triangles in the source triangle mesh
								const HaU32 *indices, // The triangle indices in the format of i1,i2,i3 ... i4,i5,i6, ...
								HaU32 maxDepth=15,	// Maximum recursion depth for the triangle mesh.
								HaU32 minLeafSize=4,	// minimum triangles to treat as a 'leaf' node.
								HaF64	minAxisSize=0.01f	// once a particular axis is less than this size, stop sub-dividing.
								);


}; // end of hacd namespace

#endif
