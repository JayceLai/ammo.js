/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "PrimitiveShape.h"

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "BulletCollision/CollisionShapes/btConvexPointCloudShape.h"

struct PrimitiveShapeExample : public CommonRigidBodyBase
{
	PrimitiveShapeExample(struct GUIHelperInterface* helper)
		: CommonRigidBodyBase(helper)
	{
	}
	virtual ~PrimitiveShapeExample() {}
	virtual void initPhysics();
	virtual void renderScene();
	virtual bool keyboardCallback(int key, int state);
	void resetCamera()
	{
		float dist = 10;
		float pitch = -35;
		float yaw = 52;
		float targetPos[3] = {0, 0.46, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
};

void PrimitiveShapeExample::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();

	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);

	///create a few basic rigid bodies
	btBoxShape* groundShape = createBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));
	btCompoundShape* comShape = new btCompoundShape();
	btTransform t;
	t.setIdentity();
	comShape->addChildShape(t, groundShape);
	comShape->setMaterial(0, 0.5, 1., 0.);
	m_collisionShapes.push_back(comShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, -50, 0));
	{
		btScalar mass(0.);
		createRigidBody(mass, groundTransform, comShape, btVector4(0, 0, 1, 1));
	}

	//BOX
	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance
		btBoxShape* box = createBoxShape(btVector3(1, 1, 1));
		btCompoundShape* colShape = new btCompoundShape();
		btTransform t;
		t.setIdentity();
		colShape->addChildShape(t, box);
		colShape->setMaterial(0, 0.5, 1., 0.);

		m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass, localInertia);

		startTransform.setOrigin(btVector3(
			btScalar(3),
			btScalar(20),
			btScalar(0)));
		createRigidBody(mass, startTransform, colShape);
	}

	//SPHERE
	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance
		btSphereShape* sphere = new btSphereShape(0.5);
		btCompoundShape* colShape = new btCompoundShape();
		btTransform t;
		t.setIdentity();
		colShape->addChildShape(t, sphere);
		colShape->setMaterial(0, 0.5, 0.5, 0.);

		int address = (int)colShape;
		btCollisionShape* p = (btCollisionShape*)address;
		p->setLocalScaling(btVector3(btScalar(1.), btScalar(1.), btScalar(1.)));

		m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass, localInertia);

		startTransform.setOrigin(btVector3(
			btScalar(0),
			btScalar(20),
			btScalar(2)));
		createRigidBody(mass, startTransform, colShape);
	}

	//CAPSULE
	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance
		btCapsuleShape* capsule = new btCapsuleShape(0.5, 1);
		btCompoundShape* colShape = new btCompoundShape();
		btTransform t;
		t.setIdentity();
		colShape->addChildShape(t, capsule);
		colShape->setMaterial(0, 0.5, 0., 0.);

		m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass, localInertia);

		startTransform.setOrigin(btVector3(
			btScalar(1),
			btScalar(10),
			btScalar(0)));
		createRigidBody(mass, startTransform, colShape);
	}

	//CYLINDER
	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance
		btCylinderShape* cylinder = new btCylinderShape(btVector3(0.5, 1, 0.5));
		btCompoundShape* colShape = new btCompoundShape();
		btTransform t;
		t.setIdentity();
		colShape->addChildShape(t, cylinder);
		colShape->setMaterial(0, 0.5, 0., 0.);

		m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass, localInertia);

		startTransform.setOrigin(btVector3(
			btScalar(4),
			btScalar(10),
			btScalar(0)));
		createRigidBody(mass, startTransform, colShape);
	}

	//CONE
	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance
		btConeShape* cone = new btConeShape(0.5, 1);
		btCompoundShape* colShape = new btCompoundShape();
		btTransform t;
		t.setIdentity();
		colShape->addChildShape(t, cone);
		colShape->setMaterial(0, 0.5, 0., 0.);

		m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass, localInertia);

		startTransform.setOrigin(btVector3(
			btScalar(3),
			btScalar(20),
			btScalar(0)));
		createRigidBody(mass, startTransform, colShape);
	}

	//CONVEX HULL
	{
		btConvexHullShape* convexHull = new btConvexHullShape();

		btScalar scaling(1);

		convexHull->addPoint(btVector3(0, 2, 0));
		convexHull->addPoint(btVector3(1, 1, 1));
		convexHull->addPoint(btVector3(1, 1, -1));
		convexHull->addPoint(btVector3(1, -1, 1));
		convexHull->addPoint(btVector3(1, -1, -1));
		convexHull->addPoint(btVector3(-1, 1, 1));
		convexHull->addPoint(btVector3(-1, 1, -1));
		convexHull->addPoint(btVector3(-1, -1, 1));
		convexHull->addPoint(btVector3(-1, -1, -1));
		
		convexHull->addPoint(btVector3(0, 0, 0));
		for (int i = 0; i < 10; i++)
		{
			btVector3 vtx(rand() % 1, rand() % 1, rand() % 1);
			convexHull->addPoint(vtx * btScalar(1. / scaling));
		}
		
		convexHull->setLocalScaling(btVector3(0.5, 1, 1));

		btCompoundShape* colShape = new btCompoundShape();
		btTransform t;
		t.setIdentity();
		colShape->addChildShape(t, convexHull);
		colShape->setMaterial(0, 0.5, 0., 0.);

		m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass, localInertia);

		startTransform.setOrigin(btVector3(
			btScalar(3),
			btScalar(20),
			btScalar(0)));
		createRigidBody(mass, startTransform, colShape);
	}

	//CONVEX TRIANGLE MESH
	{

	}

	//CONVEX POINT CLOUD
	{
		btConvexPointCloudShape* convexPC = new btConvexPointCloudShape();

		btScalar scaling(1);
		btVector3 points[20];
		int j = 0;
		points[j++]=(btVector3(0, 2, 0));
		points[j++]=(btVector3(1, 1, 1));
		points[j++]=(btVector3(1, 1, -1));
		points[j++]=(btVector3(1, -1, 1));
		points[j++]=(btVector3(1, -1, -1));
		points[j++]=(btVector3(-1, 1, 1));
		points[j++]=(btVector3(-1, 1, -1));
		points[j++]=(btVector3(-1, -1, 1));
		points[j++]=(btVector3(-1, -1, -1));
		points[j++]=(btVector3(0, 0, 0));
		for (int i = 0; i < 10; i++)
		{
			btVector3 vtx(rand() % 1, rand() % 1, rand() % 1);
			points[i++]=(vtx * btScalar(1. / scaling));
		}
		convexPC->setPoints(points, 20, true, btVector3(1.5, 1.5, 1.5));

		btCompoundShape* colShape = new btCompoundShape();
		btTransform t;
		t.setIdentity();
		colShape->addChildShape(t, convexPC);
		colShape->setMaterial(0, 0.5, 0., 0.);

		m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar mass(0.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass, localInertia);

		startTransform.setOrigin(btVector3(
			btScalar(3),
			btScalar(1),
			btScalar(0)));
		createRigidBody(mass, startTransform, colShape);
	}

	//MUTI SPHERE

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

bool PrimitiveShapeExample::keyboardCallback(int key, int state)
{
	if (key == 81)
	{
	}
	else if (key == 69)
	{
	}

	if (state)
	{
	}

	return false;
}

void PrimitiveShapeExample::renderScene()
{
	CommonRigidBodyBase::renderScene();
}

CommonExampleInterface* ET_PrimitiveShapeCreateFunc(CommonExampleOptions& options)
{
	return new PrimitiveShapeExample(options.m_guiHelper);
}
