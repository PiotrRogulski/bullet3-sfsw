#include "SpinningCube.h"

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"

struct SpinningCube : public CommonRigidBodyBase
{
	explicit SpinningCube(struct GUIHelperInterface *helper) : CommonRigidBodyBase(helper) {}
	~SpinningCube() override = default;
	void initPhysics() override;
	void renderScene() override;
	void resetCamera() override
	{
		float dist = 5;
		float pitch = -35;
		float yaw = 52;
		float targetPos[3] = {0, 0, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
};

void SpinningCube::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();
	m_dynamicsWorld->setGravity(btVector3(0, -10, 0));
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (auto *drawer = m_dynamicsWorld->getDebugDrawer())
	{
		drawer->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);
	}

	btBoxShape *boxShape = createBoxShape(btVector3(1, 1, 1));
	btTransform boxTransform;
	boxTransform.setIdentity();
	boxTransform.setOrigin(btVector3(1, 1, 1));
	btScalar mass(1);
	btRigidBody *boxBody = createRigidBody(mass, boxTransform, boxShape, btVector4(0, 0, 1, 1));
	boxBody->setAngularVelocity(10 * btVector3(1, 1, 1));

	btTransform boxAnchorTransform;
	boxAnchorTransform.setIdentity();
	boxAnchorTransform.setOrigin(btVector3(-1, -1, -1));
    auto *pivot = new btConeTwistConstraint(*boxBody, boxAnchorTransform);
	pivot->setDbgDrawSize(2);
	m_dynamicsWorld->addConstraint(pivot);

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void SpinningCube::renderScene()
{
	CommonRigidBodyBase::renderScene();
}

CommonExampleInterface *SpinningCubeCreateFunc(CommonExampleOptions &options)
{
	return new SpinningCube(options.m_guiHelper);
}
