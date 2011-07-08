#ifndef TRI_TRI_H

#define TRI_TRI_H

#include "PlatformConfig.h"

int tri_tri_overlap_test_3d(physx::PxF32 p1[3], physx::PxF32 q1[3], physx::PxF32 r1[3],
			    physx::PxF32 p2[3], physx::PxF32 q2[3], physx::PxF32 r2[3]);


int coplanar_tri_tri3d(physx::PxF32  p1[3], physx::PxF32  q1[3], physx::PxF32  r1[3],
		       physx::PxF32  p2[3], physx::PxF32  q2[3], physx::PxF32  r2[3],
		       physx::PxF32  N1[3], physx::PxF32  N2[3]);


int tri_tri_overlap_test_2d(physx::PxF32 p1[2], physx::PxF32 q1[2], physx::PxF32 r1[2],
			    physx::PxF32 p2[2], physx::PxF32 q2[2], physx::PxF32 r2[2]);


int tri_tri_intersection_test_3d(physx::PxF32 p1[3], physx::PxF32 q1[3], physx::PxF32 r1[3],
				 physx::PxF32 p2[3], physx::PxF32 q2[3], physx::PxF32 r2[3],
				 int * coplanar,
				 physx::PxF32 source[3],physx::PxF32 target[3]);

#endif
