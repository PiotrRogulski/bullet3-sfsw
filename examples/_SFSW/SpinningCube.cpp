#include "SpinningCube.h"

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"

struct SpinningCube : public CommonRigidBodyBase {
    SpinningCube(struct GUIHelperInterface* helper) : CommonRigidBodyBase(helper) {}
    virtual ~SpinningCube() {}
    virtual void initPhysics();
    virtual void renderScene();
    void resetCamera() {
        float dist = 5;
        float pitch = -35;
        float yaw = 52;
        float targetPos[3] = {0, 0, 0};
        m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
    }
};

void SpinningCube::initPhysics() {
    m_guiHelper->setUpAxis(1);

    createEmptyDynamicsWorld();
    m_dynamicsWorld->setGravity(btVector3(0, -10, 0));
    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

    if (auto *drawer = m_dynamicsWorld->getDebugDrawer()) {
        drawer->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);
    }

    btBoxShape *boxShape = createBoxShape(btVector3(1, 1, 1));
    btTransform boxTransform;
    boxTransform.setIdentity();
    boxTransform.setOrigin(btVector3(1, 1, 1));
    btScalar mass(1);
    btRigidBody *boxBody = createRigidBody(mass, boxTransform, boxShape, btVector4(0, 0, 1, 1));

    btSphereShape *anchorShape = new btSphereShape(0.1);
    btTransform anchorTransform;
    anchorTransform.setIdentity();
    anchorTransform.setOrigin(btVector3(0, 0, 0));
    btScalar anchorMass(0);
    btRigidBody *anchorBody = createRigidBody(anchorMass, anchorTransform, anchorShape, btVector4(1, 0, 0, 1));

    btTransform boxAnchorTransform;
    boxAnchorTransform.setIdentity();
    boxAnchorTransform.setOrigin(btVector3(-1, -1, -1));
    // btConeTwistConstraint *pivot = new btConeTwistConstraint(*boxBody, *anchorBody, boxAnchorTransform, btTransform::getIdentity());
    btPoint2PointConstraint *pivot = new btPoint2PointConstraint(*boxBody, *anchorBody, btVector3(-1, -1, -1), btVector3(0, 0, 0));
    pivot->setDbgDrawSize(2);
    pivot->setBreakingImpulseThreshold(1000000000);
    m_dynamicsWorld->addConstraint(pivot);

    m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void SpinningCube::renderScene() {
    CommonRigidBodyBase::renderScene();
}

CommonExampleInterface* SpinningCubeCreateFunc(CommonExampleOptions& options) {
    return new SpinningCube(options.m_guiHelper);
}
