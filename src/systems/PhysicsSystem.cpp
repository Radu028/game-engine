#include "systems/PhysicsSystem.h"

#include <algorithm>

#include "GameWorld.h"
#include "entities/HumanoidCharacter.h"
#include "objects/CubeObject.h"
#include "objects/Floor.h"
#include "objects/GameObject.h"
#include "objects/Sphere.h"
#include "settings/Physics.h"

PhysicsSystem::PhysicsSystem(GameWorld* gameWorld) : world(gameWorld) {
  collisionConfig = new btDefaultCollisionConfiguration();
  dispatcher = new btCollisionDispatcher(collisionConfig);
  broadphase = new btDbvtBroadphase();
  solver = new btSequentialImpulseConstraintSolver();
  dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver,
                                              collisionConfig);
  dynamicsWorld->setGravity(
      btVector3(0, GameSettings::Physics::GRAVITY_ACCELERATION, 0));
}

PhysicsSystem::~PhysicsSystem() {
  delete dynamicsWorld;
  delete solver;
  delete broadphase;
  delete dispatcher;
  delete collisionConfig;
}

void PhysicsSystem::addObject(GameObject* obj) {
  btCollisionShape* shape = nullptr;
  if (auto* cube = dynamic_cast<CubeObject*>(obj)) {
    Vector3 sz = cube->getSize();
    float halfX = sz.x * 0.5f;
    float halfY = sz.y * 0.5f;
    float halfZ = sz.z * 0.5f;
    shape = new btBoxShape(btVector3(halfX, halfY, halfZ));
  } else if (auto* floor = dynamic_cast<Floor*>(obj)) {
    BoundingBox bbox = floor->getBoundingBox();
    Vector3 dims = {bbox.max.x - bbox.min.x, bbox.max.y - bbox.min.y,
                    bbox.max.z - bbox.min.z};
    float halfX = dims.x * 0.5f;
    float halfY = dims.y * 0.5f;
    float halfZ = dims.z * 0.5f;
    shape = new btBoxShape(btVector3(halfX, halfY, halfZ));
  } else if (auto* sphere = dynamic_cast<Sphere*>(obj)) {
    float radius = sphere->getRadius();
    shape = new btSphereShape(radius);
  } else {
    shape = new btBoxShape(btVector3(0.5f, 0.5f, 0.5f));
  }
  btDefaultMotionState* motionState = new btDefaultMotionState(
      btTransform(btQuaternion(0, 0, 0, 1),
                  btVector3(obj->getPosition().x, obj->getPosition().y,
                            obj->getPosition().z)));
  btScalar mass = obj->getIsStatic() ? 0.0f : 1.0f;
  btVector3 inertia(0, 0, 0);
  if (mass != 0.0f) shape->calculateLocalInertia(mass, inertia);
  btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motionState, shape,
                                                  inertia);
  btRigidBody* body = new btRigidBody(rbInfo);
  body->setActivationState(DISABLE_DEACTIVATION);
  if (dynamic_cast<Floor*>(obj)) {
    body->setFriction(1.5f);
    body->setRollingFriction(1.5f);
    body->setSpinningFriction(1.5f);
    body->setDamping(0.0f, 0.0f);
    body->setCollisionFlags(body->getCollisionFlags() |
                            btCollisionObject::CF_STATIC_OBJECT);
  } else if (dynamic_cast<CubeObject*>(obj)) {
    body->setFriction(2.0f);
    body->setRollingFriction(2.0f);
    body->setSpinningFriction(2.0f);
    body->setDamping(0.8f, 0.8f);
  }

  short worldGroup = GameSettings::Collision::Groups::WORLD_OBJECTS;
  short worldMask = GameSettings::Collision::Groups::WORLD_MASK;
  dynamicsWorld->addRigidBody(body, worldGroup, worldMask);
  obj->setBulletBody(body);
  objectToBody[obj] = body;
  physicsObjects.push_back(obj);
}

void PhysicsSystem::removeObject(GameObject* obj) {
  auto it = objectToBody.find(obj);
  if (it != objectToBody.end()) {
    btRigidBody* body = it->second;
    if (body) {
      dynamicsWorld->removeRigidBody(body);
      delete body->getMotionState();
      delete body->getCollisionShape();
      delete body;
      obj->setBulletBody(nullptr);
    }
    objectToBody.erase(it);
  }
  physicsObjects.erase(
      std::remove(physicsObjects.begin(), physicsObjects.end(), obj),
      physicsObjects.end());
}

void PhysicsSystem::update(float deltaTime) {
  const float fixedTimeStep = 1.0f / 60.0f;
  const int maxSubSteps = 10;

  dynamicsWorld->stepSimulation(deltaTime, maxSubSteps, fixedTimeStep);
  syncGameObjectsFromBullet();
}

void PhysicsSystem::syncGameObjectsFromBullet() {
  for (GameObject* obj : physicsObjects) {
    if (dynamic_cast<class HumanoidCharacter*>(obj)) {
      continue;
    }

    btRigidBody* body = obj->getBulletBody();
    if (body) {
      btTransform trans;
      body->getMotionState()->getWorldTransform(trans);
      const btVector3& pos = trans.getOrigin();
      obj->setPosition({pos.x(), pos.y(), pos.z()});
    }
  }
}