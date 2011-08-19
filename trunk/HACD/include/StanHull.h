#ifndef STAN_HULL_H

#define STAN_HULL_H

/*----------------------------------------------------------------------
		Copyright (c) 2004 Open Dynamics Framework Group
					www.physicstools.org
		All rights reserved.

		Redistribution and use in source and binary forms, with or without modification, are permitted provided
		that the following conditions are met:

		Redistributions of source code must retain the above copyright notice, this list of conditions
		and the following disclaimer.

		Redistributions in binary form must reproduce the above copyright notice,
		this list of conditions and the following disclaimer in the documentation
		and/or other materials provided with the distribution.

		Neither the name of the Open Dynamics Framework Group nor the names of its contributors may
		be used to endorse or promote products derived from this software without specific prior written permission.

		THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES,
		INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
		DISCLAIMED. IN NO EVENT SHALL THE INTEL OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
		EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
		LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
		IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
		THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
-----------------------------------------------------------------------*/

#include "PlatformConfigHACD.h"

namespace hacd
{

class HullResult
{
public:
	HullResult(void)
	{
		mNumOutputVertices = 0;
		mOutputVertices = 0;
		mNumFaces = 0;
		mNumIndices = 0;
		mIndices = 0;
	}
	HaU32            mNumOutputVertices;         // number of vertices in the output hull
	HaF32			*mOutputVertices;            // array of vertices, 3 floats each x,y,z
	HaU32            mNumFaces;                  // the number of faces produced
	HaU32            mNumIndices;                // the total number of indices
	HaU32           *mIndices;                   // pointer to indices.

// If triangles, then indices are array indexes into the vertex list.
// If polygons, indices are in the form (number of points in face) (p1, p2, p3, ..) etc..
};

bool createConvexHull(HaU32 vcount,const HaF32 *vertices,HullResult &result,HaU32 maxHullVertices=256,HaF32 hullSkinWidth=0);
void releaseHullResult(HullResult &result); // release memory allocated for this result, we are done with it.

}; // end of namespace

#endif
