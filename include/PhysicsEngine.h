#ifndef PHYSICS_ENGINE_H
#define PHYSICS_ENGINE_H

#include <btBulletDynamicsCommon.h>

#include <memory>

#include "Physics.h"

class PhysicsEngine {
 private:
  std::unique_ptr<btBroadphaseInterface> broadphase;
  std::unique_ptr<btDefaultCollisionConfiguration> collisionConfig;
  std::unique_ptr<btCollisionDispatcher> dispatcher;
  std::unique_ptr<btSequentialImpulseConstraintSolver> solver;
  std::unique_ptr<btDiscreteDynamicsWorld> dynamicsWorld;

  PhysicsEngine();

 public:
  ~PhysicsEngine();

  static PhysicsEngine& getInstance();

  void stepSimulation(float deltaTime);

  btDiscreteDynamicsWorld* getDynamicsWorld() const;
};

#endif