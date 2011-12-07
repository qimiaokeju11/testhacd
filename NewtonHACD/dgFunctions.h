#ifndef DG_FUNCTIONS_H

#define DG_FUNCTIONS_H

#include "dgTypes.h"

class dgCollision;
class dgBody;
class dgMatrix;
class dgPlane;

typedef hacd::HaU32 (dgApi *OnRayPrecastAction) (const dgBody* const body, const dgCollision* const collision, void* const userData);
typedef hacd::HaF32 (dgApi *OnRayCastAction) (const dgBody* const body, const dgVector& normal, hacd::HaI32 collisionID, void* const userData, hacd::HaF32 intersetParam);
typedef hacd::HaU32 (dgApi *GetBuoyancyPlane) (void* collisionID, void* context, const dgMatrix& matrix, dgPlane& plane);



#endif
