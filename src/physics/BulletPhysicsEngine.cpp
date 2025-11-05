#include "physics/BulletPhysicsEngine.h"

#include <algorithm>

#include "objects/GameObject.h"
#include "physics/PhysicsTypes.h"
#include "settings/Physics.h"

namespace {
constexpr float kDefaultMass = 1.0f;
}

BulletPhysicsEngine::BulletPhysicsEngine()
    : collisionConfig(new btDefaultCollisionConfiguration()),
      dispatcher(new btCollisionDispatcher(collisionConfig)),
      broadphase(new btDbvtBroadphase()),
      solver(new btSequentialImpulseConstraintSolver()),
      dynamicsWorld(new btDiscreteDynamicsWorld(dispatcher, broadphase, solver,
                                                collisionConfig)) {
  dynamicsWorld->setGravity(
      btVector3(0, GameSettings::Physics::GRAVITY_ACCELERATION, 0));
}

BulletPhysicsEngine::~BulletPhysicsEngine() {
  for (auto& entry : bodies) {
    if (!entry.body) continue;
    dynamicsWorld->removeRigidBody(entry.body);
    if (entry.object) entry.object->setBulletBody(nullptr);
    delete entry.body->getMotionState();
    delete entry.body->getCollisionShape();
    delete entry.body;
  }

  delete dynamicsWorld;
  delete solver;
  delete broadphase;
  delete dispatcher;
  delete collisionConfig;
}

void BulletPhysicsEngine::addObject(GameObject& object) {
  if (bodyLookup.count(&object) != 0) return;

  const PhysicsBodyConfig config = object.getPhysicsConfig();
  if (!config.usesPhysics) return;

  btRigidBody* body = createBody(object, config);
  if (!body) return;

  const short group = GameSettings::Collision::Groups::WORLD_OBJECTS;
  const short mask = GameSettings::Collision::Groups::WORLD_MASK;
  dynamicsWorld->addRigidBody(body, group, mask);

  object.setBulletBody(body);
  bodies.push_back({&object, body});
  bodyLookup[&object] = body;
}

void BulletPhysicsEngine::removeObject(GameObject& object) {
  auto it = bodyLookup.find(&object);
  if (it == bodyLookup.end()) return;

  btRigidBody* body = it->second;
  if (body) {
    dynamicsWorld->removeRigidBody(body);
    delete body->getMotionState();
    delete body->getCollisionShape();
    delete body;
  }

  object.setBulletBody(nullptr);
  bodyLookup.erase(it);

  bodies.erase(std::remove_if(bodies.begin(), bodies.end(),
                              [&](const BodyEntry& entry) {
                                return entry.object == &object;
                              }),
               bodies.end());
}

void BulletPhysicsEngine::update(float deltaTime) {
  const float fixedTimeStep = 1.0f / 60.0f;
  const int maxSubSteps = 10;
  dynamicsWorld->stepSimulation(deltaTime, maxSubSteps, fixedTimeStep);

  for (auto& entry : bodies) {
    if (!entry.object || !entry.body) continue;
    syncFromBullet(*entry.object, *entry.body);
  }
}

btDiscreteDynamicsWorld* BulletPhysicsEngine::getNativeWorld() {
  return dynamicsWorld;
}

void BulletPhysicsEngine::syncFromBullet(GameObject& object,
                                         btRigidBody& body) const {
  btTransform transform;
  body.getMotionState()->getWorldTransform(transform);
  const btVector3& origin = transform.getOrigin();
  object.setPosition({origin.x(), origin.y(), origin.z()});
}

btRigidBody* BulletPhysicsEngine::createBody(GameObject& object,
                                             const PhysicsBodyConfig& config) {
  btCollisionShape* shape = createShape(config);
  if (!shape) return nullptr;

  btVector3 inertia(0, 0, 0);
  const bool dynamicBody = !config.isStatic;
  const btScalar mass = dynamicBody ? kDefaultMass : 0.0f;
  if (dynamicBody) shape->calculateLocalInertia(mass, inertia);

  btTransform startTransform;
  startTransform.setIdentity();
  const Vector3 pos = object.getPosition();
  startTransform.setOrigin(btVector3(pos.x, pos.y, pos.z));

  auto* motionState = new btDefaultMotionState(startTransform);
  btRigidBody::btRigidBodyConstructionInfo info(mass, motionState, shape,
                                                inertia);
  auto* body = new btRigidBody(info);
  object.configurePhysicsBody(*body);
  return body;
}

btCollisionShape* BulletPhysicsEngine::createShape(
    const PhysicsBodyConfig& config) const {
  switch (config.collider) {
    case ColliderType::Box: {
      const btVector3 halfExtents(config.dimensions.x * 0.5f,
                                  config.dimensions.y * 0.5f,
                                  config.dimensions.z * 0.5f);
      return new btBoxShape(halfExtents);
    }
    case ColliderType::Sphere: {
      return new btSphereShape(config.radius);
    }
    case ColliderType::None:
    default:
      return nullptr;
  }
}


