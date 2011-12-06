#ifndef DG_FUNCTIONS_H

#define DG_FUNCTIONS_H

#include "dgTypes.h"

class dgCollision;
class dgBody;
class dgMatrix;
class dgPlane;

typedef dgUnsigned32 (dgApi *OnRayPrecastAction) (const dgBody* const body, const dgCollision* const collision, void* const userData);
typedef dgFloat32 (dgApi *OnRayCastAction) (const dgBody* const body, const dgVector& normal, dgInt32 collisionID, void* const userData, dgFloat32 intersetParam);
typedef dgUnsigned32 (dgApi *GetBuoyancyPlane) (void* collisionID, void* context, const dgMatrix& matrix, dgPlane& plane);



#endif
