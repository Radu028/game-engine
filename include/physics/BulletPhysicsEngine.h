#pragma once

#include <btBulletDynamicsCommon.h>

#include <unordered_map>
#include <vector>

#include "physics/PhysicsEngine.h"
#include "physics/PhysicsTypes.h"

class BulletPhysicsEngine : public PhysicsEngine {
 public:
  BulletPhysicsEngine();
  ~BulletPhysicsEngine() override;

  void addObject(GameObject& object) override;
  void removeObject(GameObject& object) override;
  void update(float deltaTime) override;
  btDiscreteDynamicsWorld* getNativeWorld() override;

 private:
  struct BodyEntry {
    GameObject* object;
    btRigidBody* body;
  };

  btDefaultCollisionConfiguration* collisionConfig;
  btCollisionDispatcher* dispatcher;
  btBroadphaseInterface* broadphase;
  btSequentialImpulseConstraintSolver* solver;
  btDiscreteDynamicsWorld* dynamicsWorld;

  std::vector<BodyEntry> bodies;
  std::unordered_map<GameObject*, btRigidBody*> bodyLookup;

  void syncFromBullet(GameObject& object, btRigidBody& body) const;
  btRigidBody* createBody(GameObject& object, const PhysicsBodyConfig& config);
  btCollisionShape* createShape(const PhysicsBodyConfig& config) const;
};
