#ifndef PHYSICSSYSTEM_H
#define PHYSICSSYSTEM_H

#include <btBulletDynamicsCommon.h>

#include <unordered_map>
#include <vector>

#include "objects/GameObject.h"

class GameWorld;

class PhysicsSystem {
 private:
  std::vector<GameObject*> physicsObjects;
  std::unordered_map<GameObject*, btRigidBody*>
      objectToBody;  // mapare GameObject -> btRigidBody
  GameWorld* world;

 public:
  PhysicsSystem(GameWorld* gameWorld);
  ~PhysicsSystem();

  void addObject(GameObject* obj);
  void removeObject(GameObject* obj);
  void update(float deltaTime);

  // Expose Bullet world for GameWorld
  friend class GameWorld;

 private:
  void syncGameObjectsFromBullet();

  // Bullet Physics
  btDefaultCollisionConfiguration* collisionConfig;
  btCollisionDispatcher* dispatcher;
  btBroadphaseInterface* broadphase;
  btSequentialImpulseConstraintSolver* solver;
  btDiscreteDynamicsWorld* dynamicsWorld;
};

#endif
