#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>

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



#include "ConstraintBuilder.h"
#include "TriTri.h"

typedef STDNAME::vector< Constraint * > ConstraintVector;

/* a = b - c */
#define rayvector(a,b,c) \
    (a)[0] = (b)[0] - (c)[0];	\
    (a)[1] = (b)[1] - (c)[1];	\
    (a)[2] = (b)[2] - (c)[2];



#define rayInnerProduct(v,q) \
        ((v)[0] * (q)[0] + \
        (v)[1] * (q)[1] + \
        (v)[2] * (q)[2])

#define rayCrossProduct(a,b,c) \
    (a)[0] = (b)[1] * (c)[2] - (c)[1] * (b)[2]; \
    (a)[1] = (b)[2] * (c)[0] - (c)[2] * (b)[0]; \
    (a)[2] = (b)[0] * (c)[1] - (c)[0] * (b)[1];

static bool rayIntersectsTriangle(const physx::PxF32 *p,const physx::PxF32 *d,const physx::PxF32 *v0,const physx::PxF32 *v1,const physx::PxF32 *v2,physx::PxF32 &t)
{

    physx::PxF32 e1[3],e2[3],h[3],s[3],q[3];
    physx::PxF32 a,f,u,v;

    rayvector(e1,v1,v0);
    rayvector(e2,v2,v0);
    rayCrossProduct(h,d,e2);
    a = rayInnerProduct(e1,h);

    if (a > -0.00001 && a < 0.00001)
        return(false);

    f = 1/a;
    rayvector(s,p,v0);
    u = f * (rayInnerProduct(s,h));

    if (u < 0.0 || u > 1.0)
        return(false);

    rayCrossProduct(q,s,e1);
    v = f * rayInnerProduct(d,q);
    if (v < 0.0 || u + v > 1.0)
        return(false);
    // at this stage we can compute t to find out where
    // the intersection point is on the line
    t = f * rayInnerProduct(e2,q);
    if (t > 0) // ray intersection
        return(true);
    else // this means that there is a line intersection
         // but not a ray intersection
         return (false);
}


static bool lineIntersectsTriangle(const physx::PxF32 *rayStart,const physx::PxF32 *rayEnd,const physx::PxF32 *p1,const physx::PxF32 *p2,const physx::PxF32 *p3,physx::PxF32 *sect)
{
    physx::PxF32 dir[3];

  dir[0] = rayEnd[0] - rayStart[0];
  dir[1] = rayEnd[1] - rayStart[1];
  dir[2] = rayEnd[2] - rayStart[2];

  physx::PxF32 d = sqrtf(dir[0]*dir[0] + dir[1]*dir[1] + dir[2]*dir[2]);
  physx::PxF32 r = 1.0f / d;

  dir[0]*=r;
  dir[1]*=r;
  dir[2]*=r;


  physx::PxF32 t;

    bool ret = rayIntersectsTriangle(rayStart, dir, p1, p2, p3, t );

    if ( ret )
    {
        if ( t < d )
        {
            sect[0] = rayStart[0] + dir[0]*t;
            sect[1] = rayStart[1] + dir[1]*t;
            sect[2] = rayStart[2] + dir[2]*t;
        }
        else
        {
            ret = false;
        }
    }

  return ret;
}


static physx::PxF32 fm_computePlane(const physx::PxF32 *A,const physx::PxF32 *B,const physx::PxF32 *C,physx::PxF32 *n) // returns D
{
    physx::PxF32 vx = (B[0] - C[0]);
    physx::PxF32 vy = (B[1] - C[1]);
    physx::PxF32 vz = (B[2] - C[2]);

    physx::PxF32 wx = (A[0] - B[0]);
    physx::PxF32 wy = (A[1] - B[1]);
    physx::PxF32 wz = (A[2] - B[2]);

    physx::PxF32 vw_x = vy * wz - vz * wy;
    physx::PxF32 vw_y = vz * wx - vx * wz;
    physx::PxF32 vw_z = vx * wy - vy * wx;

    physx::PxF32 mag = sqrtf((vw_x * vw_x) + (vw_y * vw_y) + (vw_z * vw_z));

    if ( mag < 0.000001f )
    {
        mag = 0;
    }
    else
    {
        mag = 1.0f/mag;
    }

    physx::PxF32 x = vw_x * mag;
    physx::PxF32 y = vw_y * mag;
    physx::PxF32 z = vw_z * mag;


    physx::PxF32 D = 0.0f - ((x*A[0])+(y*A[1])+(z*A[2]));

  n[0] = x;
  n[1] = y;
  n[2] = z;

    return D;
}


class ConstrainedHull : public physx::UserAllocated
{
public:
  ConstrainedHull(physx::PxU32 vcount,const physx::PxF32 *vertices,physx::PxU32 tcount,const physx::PxU32 *indices,physx::PxF32 volume,physx::PxU32 userData)
  {
    mVcount = vcount;
    mTcount = tcount;
    mVertices = (physx::PxF32 *)PX_ALLOC(sizeof(physx::PxF32)*vcount*3);
    mIndices  = (physx::PxU32 *)PX_ALLOC(sizeof(physx::PxU32)*tcount*3);
    memcpy(mVertices,vertices,sizeof(physx::PxF32)*vcount*3);
    memcpy(mIndices,indices,sizeof(physx::PxU32)*tcount*3);
    mVolume   = volume;
    mUserData = userData;

    physx::PxF32 bmin[3],bmax[3];

    bmin[0] = bmax[0] = vertices[0];
    bmin[1] = bmax[1] = vertices[1];
    bmin[2] = bmax[2] = vertices[2];

    const physx::PxF32 *scan = vertices+3;
    for (physx::PxU32 i=1; i<vcount; i++)
    {
      if ( scan[0] < bmin[0] ) bmin[0] = scan[0];
      if ( scan[1] < bmin[1] ) bmin[1] = scan[1];
      if ( scan[2] < bmin[2] ) bmin[2] = scan[2];

      if ( scan[0] > bmax[0] ) bmax[0] = scan[0];
      if ( scan[1] > bmax[1] ) bmax[1] = scan[1];
      if ( scan[2] > bmax[2] ) bmax[2] = scan[2];
      scan+=3;

    }

    mCenter[0] = ((bmax[0] - bmin[0])*0.5f)+bmin[0];
    mCenter[1] = ((bmax[1] - bmin[1])*0.5f)+bmin[1];
    mCenter[2] = ((bmax[2] - bmin[2])*0.5f)+bmin[2];

    mPlanes = (physx::PxF32 *)PX_ALLOC(sizeof(physx::PxF32)*tcount*4);

    physx::PxU32 *idx = mIndices;

    physx::PxF32 *plane = mPlanes;
    for (physx::PxU32 i=0; i<mTcount; i++)
    {
      physx::PxU32 i1 = *idx++;
      physx::PxU32 i2 = *idx++;
      physx::PxU32 i3 = *idx++;

      const physx::PxF32 *p1 = &mVertices[i1*3];
      const physx::PxF32 *p2 = &mVertices[i2*3];
      const physx::PxF32 *p3 = &mVertices[i3*3];

      plane[3] = fm_computePlane(p1,p2,p3,plane);

      plane+=4;
    }


    mUsed = false;

  }

  ~ConstrainedHull(void)
  {
    PX_FREE(mVertices);
    PX_FREE(mIndices);
    PX_FREE(mPlanes);
  }

  physx::PxU32 getUserData(void) const { return mUserData; };

  bool rayCast(const physx::PxF32 *rayStart,const physx::PxF32 *rayEnd,physx::PxF32 *intersect)
  {
    bool ret = false;
    physx::PxF32 nearest = 1e9;

    physx::PxU32 *idx = mIndices;

    for (physx::PxU32 i=0; i<mTcount; i++)
    {
      physx::PxU32 i1 = *idx++;
      physx::PxU32 i2 = *idx++;
      physx::PxU32 i3 = *idx++;

      const physx::PxF32 *p1 = &mVertices[i1*3];
      const physx::PxF32 *p2 = &mVertices[i2*3];
      const physx::PxF32 *p3 = &mVertices[i3*3];

      physx::PxF32 sect[3];

      bool hit = lineIntersectsTriangle(rayStart,rayEnd,p1,p2,p3,sect);

      if ( hit )
      {

        physx::PxF32 dx = sect[0] - rayStart[0];
        physx::PxF32 dy = sect[1] - rayStart[1];
        physx::PxF32 dz = sect[2] - rayStart[2];

        physx::PxF32 dist = dx*dx+dy*dy+dz*dz;

        if ( dist < nearest )
        {
          ret = true;
          intersect[0] = sect[0];
          intersect[1] = sect[1];
          intersect[2] = sect[2];
          nearest = dist;
        }
      }
    }
    return ret;
  }

  bool coplaner(const physx::PxF32 *plane1,const physx::PxF32 *plane2)
  {

        const physx::PxF32 PTHRESH=0.3f;

    bool ret = false;

    physx::PxF32 dx = fabsf(plane1[0]+plane2[0]);
    physx::PxF32 dy = fabsf(plane1[1]+plane2[1]);
    physx::PxF32 dz = fabsf(plane1[2]+plane2[2]);

    physx::PxF32 diff = dx*dx+dy*dy+dz*dz;

    if ( diff < (0.3f*0.3f) )
    {
            physx::PxF32 d1 = plane1[3];
            physx::PxF32 d2 = plane2[3];

            physx::PxF32 d = fabsf(d1+d2);

            if ( d < PTHRESH )
          {
        ret = true;
      }
    }

    return ret;
  }

  void getTri(const physx::PxU32 *idx,const physx::PxF32 *verts,physx::PxF32 *t1,physx::PxF32 *t2,physx::PxF32 *t3)
  {
    physx::PxU32 i1 = *idx++;
    physx::PxU32 i2 = *idx++;
    physx::PxU32 i3 = *idx++;
    const physx::PxF32 *p1 = &verts[i1*3];
    const physx::PxF32 *p2 = &verts[i2*3];
    const physx::PxF32 *p3 = &verts[i3*3];

    t1[0] = p1[0];
    t1[1] = p1[1];
    t1[2] = p1[2];

    t2[0] = p2[0];
    t2[1] = p2[1];
    t2[2] = p2[2];

    t3[0] = p3[0];
    t3[1] = p3[1];
    t3[2] = p3[2];
  }

  void copy(physx::PxF32 *dest,const physx::PxF32 *source)
  {
    dest[0] = source[0];
    dest[1] = source[1];
    dest[2] = source[2];
  }

  void include(physx::PxF32 *bmin,physx::PxF32 *bmax,physx::PxF32 *p)
  {
    if ( p[0] < bmin[0] ) bmin[0] = p[0];
    if ( p[1] < bmin[1] ) bmin[1] = p[1];
    if ( p[2] < bmin[2] ) bmin[2] = p[2];
    if ( p[0] > bmax[0] ) bmax[0] = p[0];
    if ( p[1] > bmax[1] ) bmax[1] = p[1];
    if ( p[2] > bmax[2] ) bmax[2] = p[2];
  }

  void computeCenter(physx::PxF32 *center,const physx::PxF32 *bmin,const physx::PxF32 *bmax)
  {
    center[0] = (bmax[0]-bmin[0])*0.5f + bmin[0];
    center[1] = (bmax[1]-bmin[1])*0.5f + bmin[1];
    center[2] = (bmax[2]-bmin[2])*0.5f + bmin[2];
  }

  bool sharesEdge(ConstrainedHull *parent,physx::PxF32 *sect)
  {
    bool ret = false;

    physx::PxF32 *plane = mPlanes;
    const physx::PxU32 *idx   = mIndices;

    physx::PxF32 bmin[3];
    physx::PxF32 bmax[3];

    for (physx::PxU32 i=0; i<mTcount; i++)
    {
      physx::PxF32 *parent_plane = parent->mPlanes;
      const physx::PxU32 *parent_idx = parent->mIndices;
      for (physx::PxU32 j=0; j<parent->mTcount; j++)
      {
        if ( coplaner(plane,parent_plane) )
        {

          physx::PxF32 p1[3],p2[3],p3[3];
          physx::PxF32 tp1[3],tp2[3],tp3[3];

          getTri(parent_idx,parent->mVertices,p1,p2,p3);
          getTri(idx,mVertices,tp1,tp2,tp3);

          physx::PxI32 hit = coplanar_tri_tri3d(p1,p2,p3,tp1,tp2,tp2,parent_plane,plane);

          if ( hit )
          {
            if (!ret)
            {
              copy(bmin,p1);
              copy(bmax,p1);
            }
            include(bmin,bmax,p2);
            include(bmin,bmax,p3);
            include(bmin,bmax,tp1);
            include(bmin,bmax,tp2);
            include(bmin,bmax,tp3);
            ret = true;
          }
        }
        parent_plane+=4;
        parent_idx+=3;
      }
      plane+=4;
      idx+=3;
    }
    if ( ret )
    {
      computeCenter(sect,bmin,bmax);
    }
    return ret;
  }


  physx::PxF32         mVolume;
  physx::PxU32  mVcount;
  physx::PxF32        *mVertices;
  physx::PxU32  mTcount;
  physx::PxU32 *mIndices;
  physx::PxU32  mUserData;
  physx::PxF32         mCenter[3];
  bool          mUsed;
  physx::PxF32        *mPlanes;
};

typedef STDNAME::vector< ConstrainedHull * > ConstrainedHullVector;

class ConstraintBuilder : public physx::UserAllocated
{
public:
  ConstraintBuilder(void)
  {
  }

  ~ConstraintBuilder(void)
  {
    ConstrainedHullVector::iterator i;
    for (i=mHulls.begin(); i!=mHulls.end(); ++i)
    {
      ConstrainedHull *ch = (*i);
      delete ch;
    }
    {
      ConstraintVector::iterator i;
      for (i=mConstraints.begin(); i!=mConstraints.end(); ++i)
      {
        Constraint *c = (*i);
        delete c;
      }
    }
  }

  ConstrainedHull * addConvexHull(physx::PxU32 vcount,const physx::PxF32 *vertices,physx::PxU32 tcount,const physx::PxU32 *indices,physx::PxF32 volume,physx::PxU32 userData)
  {
    ConstrainedHull *ch = PX_NEW(ConstrainedHull)(vcount,vertices,tcount,indices,volume,userData);
    mHulls.push_back(ch);
    return ch;
  }

  physx::PxU32 getConstrainedHullCount(void)
  {
    return (physx::PxU32)mHulls.size();
  }

  ConstrainedHull * getConstrainedHull(physx::PxU32 index)
  {
    ConstrainedHull *ret = 0;
    if ( index < mHulls.size() )
      ret = mHulls[index];
    return ret;
  }

  ConstrainedHull * getLargestHull(void)
  {
    ConstrainedHull *ret = 0;
    physx::PxF32 maxV = 0;

    ConstrainedHullVector::iterator i;
    for (i=mHulls.begin(); i!=mHulls.end(); ++i)
    {
      ConstrainedHull *ch = (*i);
      if ( !ch->mUsed && ch->mVolume > maxV )
      {
        ret = ch;
        maxV = ch->mVolume;
      }
    }
    if ( ret )
    {
      ret->mUsed = true;
    }
    return ret;
  }

  void process(ConstrainedHull *ch)
  {
    ConstrainedHullVector children;

    // ok..we now have the largest hull.
    // now we find all hulls not yet 'used' that share a surface.
    ConstrainedHullVector::iterator i;
    for (i=mHulls.begin(); i!=mHulls.end(); i++)
    {
      ConstrainedHull *child = (*i);

      physx::PxF32 sect[3];

      if ( !child->mUsed && ch->sharesEdge(child,sect) )
      {

        Constraint *c = PX_NEW(Constraint)(ch,child,sect);

        children.push_back(child);
        mConstraints.push_back(c);
      }
    }
    for (i=children.begin(); i!=children.end(); i++)
    {
      ConstrainedHull *child = (*i);
      child->mUsed = true; // make it as processed
      process(child);
    }
  }

  physx::PxU32 buildConstraints(void)
  {
    ConstrainedHull *ch = getLargestHull();

    while ( ch )
    {
      process(ch);
      ch = getLargestHull();
    }

        // ok..the constraints have to be sorted now!

      physx::PxU32 count = (physx::PxU32)mConstraints.size();
      char *used = (char *)PX_ALLOC(sizeof(char)*count);
      memset(used,0,sizeof(char)*count);

        ConstraintVector slist = mConstraints;
        mConstraints.clear();

        ConstraintVector nextlist;
        ConstraintVector newlist;

        // first add all of the root nodes
        ConstrainedHull *root = slist[0]->mParent;

        for (physx::PxU32 i=0; i<count; i++)
        {
            Constraint *node =slist[i];
            if ( node->mParent == root )
            {
                used[i] = 1;
                mConstraints.push_back(node);
                nextlist.push_back(node);
            }
        }

        physx::PxU32 ncount = (physx::PxU32)nextlist.size();

        while ( ncount )
        {
        for (physx::PxU32 i=0; i<ncount; i++)
        {
            Constraint *parent = nextlist[i];
            for (physx::PxU32 j=0; j<count; j++)
            {
                if ( !used[j] ) // if not already represented...
                {
                    Constraint *child = slist[j];
                  if ( child->mParent == parent->mChild )
                  {
                    mConstraints.push_back(child);
                    newlist.push_back(child); // becomes the new parent next time around...
                    used[j] = 1;
                  }
                }
            }
            }
            nextlist = newlist;
            newlist.clear();
            ncount = (physx::PxU32)nextlist.size();
        }

        PX_FREE(used);

    return (physx::PxU32)mConstraints.size();
  }

  Constraint        *getConstraint(physx::PxU32 index,physx::PxU32 &i1,physx::PxU32 &i2)
  {
    Constraint *ret = 0;
    if ( index < mConstraints.size() )
    {
      ret = mConstraints[index];
      i1 = ret->mParent->mUserData;
      i2 = ret->mChild->mUserData;
    }
    return ret;
  }

private:
  ConstraintVector      mConstraints;
  ConstrainedHullVector mHulls;
};



ConstraintBuilder *createConstraintBuilder(void)
{
  ConstraintBuilder *ret = PX_NEW(ConstraintBuilder);
  return ret;
}

void               releaseConstraintBuilder(ConstraintBuilder *cb)
{
  delete cb;
}

ConstrainedHull *  addConvexHull(ConstraintBuilder *cb,physx::PxU32 vcount,const physx::PxF32 *vertices,physx::PxU32 tcount,const physx::PxU32 *indices,physx::PxF32 volume,physx::PxU32 userData)
{
  return cb->addConvexHull(vcount,vertices,tcount,indices,volume,userData);
}

physx::PxU32             getUserData(ConstrainedHull *ch)
{
  return ch->getUserData();
}


physx::PxU32       getConstrainedHullCount(ConstraintBuilder *cb)
{
  return cb->getConstrainedHullCount();
}

ConstrainedHull   *getConstrainedHull(ConstraintBuilder *cb,physx::PxU32 index)
{
  return cb->getConstrainedHull(index);
}


physx::PxU32       buildConstraints(ConstraintBuilder *cb) // returns number of constraints in the skeleton.
{
  return cb->buildConstraints();
}

Constraint        *getConstraint(ConstraintBuilder *cb,physx::PxU32 index,physx::PxU32 &i1,physx::PxU32 &i2)
{
  return cb->getConstraint(index,i1,i2);
}