#include "PhysicsEngine.h"

#include <algorithm>

PhysicsEngine& PhysicsEngine::getInstance() {
  static PhysicsEngine instance;
  return instance;
}

PhysicsEngine::PhysicsEngine() {
  broadphase = std::make_unique<btDbvtBroadphase>();
  collisionConfig = std::make_unique<btDefaultCollisionConfiguration>();
  dispatcher = std::make_unique<btCollisionDispatcher>(collisionConfig.get());
  solver = std::make_unique<btSequentialImpulseConstraintSolver>();
  dynamicsWorld = std::make_unique<btDiscreteDynamicsWorld>(
      dispatcher.get(), broadphase.get(), solver.get(), collisionConfig.get());
  dynamicsWorld->setGravity(
      btVector3(0, PhysicsSettings::GRAVITY_ACCELERATION, 0));
}

void PhysicsEngine::stepSimulation(float deltaTime) {
  float dt = std::clamp(deltaTime, PhysicsSettings::MIN_DELTA_TIME,
                        PhysicsSettings::MAX_DELTA_TIME);

  dynamicsWorld->stepSimulation(dt * PhysicsSettings::TIME_SCALE, 10,
                                PhysicsSettings::MIN_DELTA_TIME);
}

btDiscreteDynamicsWorld* PhysicsEngine::getDynamicsWorld() const {
  return dynamicsWorld.get();
}

PhysicsEngine::~PhysicsEngine() {}