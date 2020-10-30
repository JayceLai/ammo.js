#include "bullet/src/btBulletDynamicsCommon.h"

#include "bullet/src/BulletCollision/CollisionDispatch/btGhostObject.h"
#include "bullet/src/BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"

/*
#include "bullet/src/BulletCollision/Gimpact/btBoxCollision.h"
#include "bullet/src/BulletCollision/Gimpact/btClipPolygon.h"
#include "bullet/src/BulletCollision/Gimpact/btContactProcessing.h"
#include "bullet/src/BulletCollision/Gimpact/btGenericPoolAllocator.h"
#include "bullet/src/BulletCollision/Gimpact/btGeometryOperations.h"
#include "bullet/src/BulletCollision/Gimpact/btGImpactBvh.h"
#include "bullet/src/BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include "bullet/src/BulletCollision/Gimpact/btGImpactMassUtil.h"
#include "bullet/src/BulletCollision/Gimpact/btGImpactQuantizedBvh.h"
#include "bullet/src/BulletCollision/Gimpact/btGImpactShape.h"
#include "bullet/src/BulletCollision/Gimpact/btQuantization.h"
#include "bullet/src/BulletCollision/Gimpact/btTriangleShapeEx.h"
*/

// Additions

class ConcreteContactResultCallback : public btCollisionWorld::ContactResultCallback // for JS callbacks through vtable customization
{
public:
  ConcreteContactResultCallback(){};
  virtual float addSingleResult(btManifoldPoint &cp, const btCollisionObject *colObj0, int partId0, int index0, const btCollisionObject *colObj1, int partId1, int index1) { return 0; };
};

struct CCFilterCallback : public btOverlapFilterCallback
{
  // return true when pairs need collision
  virtual bool needBroadphaseCollision(btBroadphaseProxy *proxy0, btBroadphaseProxy *proxy1) const
  {
    bool collides = (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
    collides = collides && (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);

    //add some additional logic here that modified 'collides'
    if (collides)
    {
      btCollisionObject *co0 = (btCollisionObject *)proxy0->m_clientObject;
      btCollisionObject *co1 = (btCollisionObject *)proxy1->m_clientObject;

      if (co0->hasContactResponse() && co1->hasContactResponse())
      {
        // collision
        if (co0->isStaticOrKinematicObject() && co1->isStaticOrKinematicObject())
          return false;
      }
      else
      {
        // trigger
        if (co0->isStaticObject() && co1->isStaticObject())
          return false;
      }
    }
    return collides;
  }
};