#include "MultiBody.h"

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"

struct MultiBody : public CommonRigidBodyBase
{
	explicit MultiBody(struct GUIHelperInterface *helper) : CommonRigidBodyBase(helper) {}
	~MultiBody() override = default;
	void initPhysics() override;
	void renderScene() override;
	void resetCamera() override
	{
		float dist = 20;
		float pitch = -35;
		float yaw = 52;
		float targetPos[3] = {0, 12, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
};

void MultiBody::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();
	m_dynamicsWorld->setGravity(btVector3(0, -10, 0));
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (auto *drawer = m_dynamicsWorld->getDebugDrawer())
	{
		drawer->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);
	}

	btBoxShape *groundShape = createBoxShape(btVector3(btScalar(50), btScalar(50), btScalar(50)));
	m_collisionShapes.push_back(groundShape);
	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, -50, 0));
	{
		btScalar mass(0);
		createRigidBody(mass, groundTransform, groundShape, btVector4(0, 0, 1, 1));
	}

	btAlignedObjectArray<btRigidBody *> bodies;

	auto *headShape = new btSphereShape(btScalar(1));
	m_collisionShapes.push_back(headShape);
	btTransform headTransform;
	headTransform.setIdentity();
	headTransform.setOrigin(btVector3(0, 20, 0));
	{
		btScalar mass(1);
		bodies.push_back(createRigidBody(mass, headTransform, headShape, btVector4(1, 0, 0, 1)));
	}

	auto *torsoShape = new btBoxShape(btVector3(1.5, 3, 1.5));
	m_collisionShapes.push_back(torsoShape);
	btTransform torsoTransform;
	torsoTransform.setIdentity();
	torsoTransform.setOrigin(btVector3(0, 16, 0));
	{
		btScalar mass(1);
		bodies.push_back(createRigidBody(mass, torsoTransform, torsoShape, btVector4(0, 1, 0, 1)));
	}

	auto *armShape = new btBoxShape(btVector3(0.5, 0.2, 0.5));
	m_collisionShapes.push_back(armShape);
	btTransform armTransform;
	armTransform.setIdentity();
	for (int i = 0; i < 10; i++)
	{
		btScalar mass(1);
		armTransform.setOrigin(btVector3(0, btScalar(19.8 - i * 0.4), 2));
		bodies.push_back(createRigidBody(mass, armTransform, armShape, btVector4(0, 1, 1, 1)));
		armTransform.setOrigin(btVector3(0, btScalar(19.8 - i * 0.4), -2));
		bodies.push_back(createRigidBody(mass, armTransform, armShape, btVector4(1, 0, 1, 1)));
	}

	auto *legShape = new btBoxShape(btVector3(0.5, 0.2, 0.5));
	m_collisionShapes.push_back(legShape);
	btTransform legTransform;
	legTransform.setIdentity();
	for (int i = 0; i < 10; i++)
	{
		btScalar mass(1);
		legTransform.setOrigin(btVector3(0, btScalar(12.8 - i * 0.4), 2));
		bodies.push_back(createRigidBody(mass, legTransform, legShape, btVector4(1, 1, 0, 1)));
		legTransform.setOrigin(btVector3(0, btScalar(12.8 - i * 0.4), -2));
		bodies.push_back(createRigidBody(mass, legTransform, legShape, btVector4(1, 1, 1, 1)));
	}

	constexpr auto sphereCount = 25;
	auto *sphereShape = new btSphereShape(btScalar(0.5));
	auto *linkShape = createBoxShape(btVector3(0.5, 0.1, 0.1));
	m_collisionShapes.push_back(sphereShape);
	m_collisionShapes.push_back(linkShape);
	btTransform sphereTransform;
	btTransform linkTransform;
	sphereTransform.setIdentity();
	linkTransform.setIdentity();
	for (int i = 0; i < sphereCount; i++)
	{
		auto sphereAngle = btScalar(i) * 2 * SIMD_PI / sphereCount;
		auto linkAngle = btScalar(i) * 2 * SIMD_PI / sphereCount + SIMD_PI / sphereCount;
		btScalar mass(1);
		sphereTransform.setOrigin(btVector3(4 * sin(sphereAngle), 25, 4 * cos(sphereAngle)));
		sphereTransform.setRotation(btQuaternion(btVector3(0, 1, 0), sphereAngle));
		linkTransform.setOrigin(btVector3(4 * sin(linkAngle), 25, 4 * cos(linkAngle)));
		linkTransform.setRotation(btQuaternion(btVector3(0, 1, 0), linkAngle));
		bodies.push_back(createRigidBody(mass, sphereTransform, sphereShape, btVector4(1, 1, 1, 1)));
		bodies.push_back(createRigidBody(mass, linkTransform, linkShape, btVector4(1, 1, 1, 1)));
	}

	auto *headTorso = new btPoint2PointConstraint(*bodies[0], *bodies[1], btVector3(0, -1, 0), btVector3(0, 3, 0));
	m_dynamicsWorld->addConstraint(headTorso);

	auto *torsoArm1 = new btPoint2PointConstraint(*bodies[1], *bodies[2], btVector3(0, 3, 1.1), btVector3(0, 0, -0.6));
	m_dynamicsWorld->addConstraint(torsoArm1);
	for (int i = 0; i < 9; i++)
	{
		auto *armArm = new btPoint2PointConstraint(*bodies[2 + i * 2], *bodies[2 + i * 2 + 2], btVector3(0, -0.2, 0), btVector3(0, 0.2, 0));
		m_dynamicsWorld->addConstraint(armArm);
	}

	auto *torsoArm2 = new btPoint2PointConstraint(*bodies[1], *bodies[3], btVector3(0, 3, -1.1), btVector3(0, 0, 0.6));
	m_dynamicsWorld->addConstraint(torsoArm2);
	for (int i = 0; i < 9; i++)
	{
		auto *armArm = new btPoint2PointConstraint(*bodies[3 + i * 2], *bodies[3 + i * 2 + 2], btVector3(0, -0.2, 0), btVector3(0, 0.2, 0));
		m_dynamicsWorld->addConstraint(armArm);
	}

	auto *torsoLeg1 = new btPoint2PointConstraint(*bodies[1], *bodies[22], btVector3(0, -3, 1), btVector3(0, 0.2, 0));
	m_dynamicsWorld->addConstraint(torsoLeg1);
	for (int i = 0; i < 9; i++)
	{
		auto *legLeg = new btPoint2PointConstraint(*bodies[22 + i * 2], *bodies[22 + i * 2 + 2], btVector3(0, -0.2, 0), btVector3(0, 0.2, 0));
		m_dynamicsWorld->addConstraint(legLeg);
	}

	auto *torsoLeg2 = new btPoint2PointConstraint(*bodies[1], *bodies[23], btVector3(0, -3, -1), btVector3(0, 0.2, 0));
	m_dynamicsWorld->addConstraint(torsoLeg2);
	for (int i = 0; i < 9; i++)
	{
		auto *legLeg = new btPoint2PointConstraint(*bodies[23 + i * 2], *bodies[23 + i * 2 + 2], btVector3(0, -0.2, 0), btVector3(0, 0.2, 0));
		m_dynamicsWorld->addConstraint(legLeg);
	}

	for (int i = 0; i < sphereCount * 2; i++)
	{
		auto *body1 = bodies[42 + i];
		auto *body2 = bodies[42 + (i + 1) % (sphereCount * 2)];
		auto *joint = new btPoint2PointConstraint(*body1, *body2, btVector3(0.5, 0, 0), btVector3(-0.5, 0, 0));
		m_dynamicsWorld->addConstraint(joint);
	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void MultiBody::renderScene()
{
	CommonRigidBodyBase::renderScene();
}

CommonExampleInterface *MultiBodyCreateFunc(CommonExampleOptions &options)
{
	return new MultiBody(options.m_guiHelper);
}