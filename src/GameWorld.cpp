#include "GameWorld.h"

#include <algorithm>

#include "ai/NPC.h"
#include "ai/NavMesh.h"
#include "objects/GameObject.h"
#include "physics/BulletPhysicsEngine.h"
#include "physics/PhysicsEngine.h"
#include "physics/PhysicsTypes.h"

GameWorld* GameWorld::instance = nullptr;

GameWorld* GameWorld::getInstance() {
  if (instance == nullptr) instance = new GameWorld();
  return instance;
}

void GameWorld::destroyInstance() {
  delete instance;
  instance = nullptr;
}

GameWorld::GameWorld(const std::string& name) : worldName(name) {
  physicsSystem = std::make_unique<BulletPhysicsEngine>();
}

GameWorld::~GameWorld() {
  for (auto& object : objects) {
    if (object) object->onRemovedFromWorld(*this);
  }
  objects.clear();

  if (instance == this) instance = nullptr;
}

void GameWorld::addObject(std::shared_ptr<GameObject> object) {
  if (!object) return;

  objects.push_back(object);
  const PhysicsBodyConfig config = object->getPhysicsConfig();
  if (physicsSystem && config.usesPhysics) physicsSystem->addObject(*object);

  if (!navigationMesh) return;
  if (!object->getIsStatic()) return;
  if (object->getObstacleType() == "npc") return;

  navigationMesh->registerObject(object.get());
}

void GameWorld::removeObject(std::shared_ptr<GameObject> object) {
  if (!object) return;

  const PhysicsBodyConfig config = object->getPhysicsConfig();
  if (physicsSystem && config.usesPhysics) physicsSystem->removeObject(*object);

  if (navigationMesh && object->getIsStatic()) {
    navigationMesh->unregisterObject(object.get());
  }

  object->onRemovedFromWorld(*this);

  objects.erase(std::remove_if(objects.begin(), objects.end(),
                               [&](const std::shared_ptr<GameObject>& entry) {
                                 return entry == object;
                               }),
                objects.end());
}

void GameWorld::update(float deltaTime) {
  for (auto& object : objects) {
    if (!object) continue;
    object->update(deltaTime);
  }

  if (physicsSystem) physicsSystem->update(deltaTime);
  if (navigationMesh) navigationMesh->updateAllRegisteredObjects();
}

void GameWorld::draw() const {
  for (const auto& object : objects) {
    if (!object) continue;
    object->draw();
  }
}

btDiscreteDynamicsWorld* GameWorld::getBulletWorld() const {
  return physicsSystem ? physicsSystem->getNativeWorld() : nullptr;
}

void GameWorld::initializeNavMesh() {
  const Vector3 minBounds{-25.0f, 0.0f, -25.0f};
  const Vector3 maxBounds{25.0f, 0.0f, 25.0f};
  const float groundLevel = 0.0f;

  navigationMesh =
      std::make_shared<NavMesh>(minBounds, maxBounds, 1.2f, groundLevel);
  navigationMesh->generateNavMesh();

  for (const auto& object : objects) {
    if (!object || !object->getIsStatic()) continue;
    navigationMesh->registerObject(object.get());
  }

  navigationMesh->rebuildConnections();
  NPC::setNavMesh(navigationMesh);
}

void GameWorld::addObjectAsObstacle(std::shared_ptr<GameObject> object,
                                    const std::string& type) {
  if (!object) return;

  addObject(object);
  if (!navigationMesh) return;
  if (!navigationMesh->shouldObjectBlockPath(object.get())) return;

  const Vector3 position = object->getPosition();
  const Vector3 size = object->getObstacleSize();
  const std::string obstacleType =
      type.empty() ? object->getObstacleType() : type;

  navigationMesh->addObstacle(position, size, obstacleType);
}

void GameWorld::removeObjectObstacle(std::shared_ptr<GameObject> object) {
  if (!object || !navigationMesh) return;

  const Vector3 position = object->getPosition();
  const Vector3 size = object->getObstacleSize();

  navigationMesh->removeObstacle(position, size);
  removeObject(object);
}

void GameWorld::addObstacleAt(Vector3 position, Vector3 size,
                              const std::string& type) {
  if (!navigationMesh) return;
  navigationMesh->addObstacle(position, size, type);
}

void GameWorld::addObjectAsObstacleDeferred(std::shared_ptr<GameObject> object,
                                            const std::string& type) {
  if (!object) return;

  addObject(object);
  if (!navigationMesh) return;
  if (!navigationMesh->shouldObjectBlockPath(object.get())) return;

  const Vector3 position = object->getPosition();
  const Vector3 size = object->getObstacleSize();
  const std::string obstacleType =
      type.empty() ? object->getObstacleType() : type;

  const float weight = obstacleType == "shelf"  ? 0.7f
                       : obstacleType == "wall" ? 0.3f
                                                : 0.5f;
  navigationMesh->markNodesInArea(position, size, false, weight);
}

void GameWorld::finalizeObstacles() {
  if (!navigationMesh) return;
  navigationMesh->rebuildConnections();
}

void GameWorld::setPhysicsEngine(std::unique_ptr<PhysicsEngine> engine) {
  if (!engine) return;

  physicsSystem = std::move(engine);
  for (auto& object : objects) {
    if (!object) continue;
    const PhysicsBodyConfig config = object->getPhysicsConfig();
    if (!config.usesPhysics) continue;
    physicsSystem->addObject(*object);
  }
}
