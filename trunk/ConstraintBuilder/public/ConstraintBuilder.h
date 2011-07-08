#ifndef CONSTRAINT_BUILDER_H

#define CONSTRAINT_BUILDER_H

/*!  
** 
** Copyright (c) 2007,2011 by John W. Ratcliff mailto:jratcliffscarab@gmail.com
**
** Portions of this source has been released with the PhysXViewer application, as well as 
** Rocket, CreateDynamics, ODF, and as a number of sample code snippets.
**
** If you wish to contact me you can use the following methods:
**
** email: jratcliffscarab@gmail.com
** Personal website: http://jratcliffscarab.blogspot.com
** Coding Website:   http://codesuppository.blogspot.com
**
**
** The MIT license:
**
** Permission is hereby granted, free of charge, to any person obtaining a copy 
** of this software and associated documentation files (the "Software"), to deal 
** in the Software without restriction, including without limitation the rights 
** to use, copy, modify, merge, publish, distribute, sublicense, and/or sell 
** copies of the Software, and to permit persons to whom the Software is furnished 
** to do so, subject to the following conditions:
**
** The above copyright notice and this permission notice shall be included in all 
** copies or substantial portions of the Software.

** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
** AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
** WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN 
** CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/


#include "PlatformConfig.h"


class ConstraintBuilder;
class ConstrainedHull;

class Constraint : public physx::UserAllocated
{
public:
  Constraint(ConstrainedHull *parent,ConstrainedHull *child,const physx::PxF32 *sect)
  {
    mParent = parent;
    mChild  = child;
    mIntersect[0] = sect[0];
    mIntersect[1] = sect[1];
    mIntersect[2] = sect[2];
  }
  ConstrainedHull *mParent;
  ConstrainedHull *mChild;
  physx::PxF32            mIntersect[3]; // the mean value of the joined surfaces.
};

ConstraintBuilder *createConstraintBuilder(void);
void               releaseConstraintBuilder(ConstraintBuilder *cb);
ConstrainedHull   *addConvexHull(ConstraintBuilder *cb,physx::PxU32 vcount,const physx::PxF32 *vertices,physx::PxU32 tcount,const physx::PxU32 *indices,physx::PxF32 volume,physx::PxU32 userData);
physx::PxU32       getUserData(ConstrainedHull *ch);
physx::PxU32       getConstrainedHullCount(ConstraintBuilder *cb);
ConstrainedHull   *getConstrainedHull(ConstraintBuilder *cb,physx::PxU32 index);
physx::PxU32       buildConstraints(ConstraintBuilder *cb); // returns number of constraints in the skeleton.
Constraint        *getConstraint(ConstraintBuilder *cb,physx::PxU32 index,physx::PxU32 &i1,physx::PxU32 &i2);


#endif
