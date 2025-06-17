#include "GameWorld.h"

#include <algorithm>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "ai/NPC.h"
#include "objects/Floor.h"
#include "shop/Shop.h"
#include "systems/PhysicsSystem.h"

GameWorld* GameWorld::instance = nullptr;
size_t GameWorld::totalObjectsCreated = 0;
size_t GameWorld::totalObjectsDestroyed = 0;
std::unordered_map<std::string, size_t> GameWorld::objectTypeCount;
std::string GameWorld::creationTimestamp;

GameWorld* GameWorld::getInstance(GameObject* player) {
  if (instance == nullptr) {
    instance = new GameWorld(player);
  }
  return instance;
}

GameWorld::GameWorld(GameObject* player, const std::string& name)
    : player(player), worldName(name) {
  // Set creation timestamp
  auto now = std::time(nullptr);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&now), "%Y-%m-%d %H:%M:%S");
  creationTimestamp = ss.str();

  physicsSystem = std::make_unique<PhysicsSystem>(this);

  if (this->player) {
    physicsSystem->addObject(this->player);
  }
}

GameWorld::~GameWorld() {
  if (instance == this) {
    instance = nullptr;
  }
}

void GameWorld::addObject(std::shared_ptr<GameObject> object) {
  if (!object) return;

  objects.push_back(object);
  if (physicsSystem) {
    physicsSystem->addObject(object.get());
  }
}

void GameWorld::removeObject(std::shared_ptr<GameObject> object) {
  if (!object) return;

  if (physicsSystem) {
    physicsSystem->removeObject(object.get());
  }

  objects.erase(std::remove_if(objects.begin(), objects.end(),
                               [&](const std::shared_ptr<GameObject>& obj) {
                                 return obj == object;
                               }),
                objects.end());
}

void GameWorld::update(float deltaTime) {
  for (auto& objSharedPtr : objects) {
    if (objSharedPtr.get() != player) {
      objSharedPtr->update(deltaTime);
    }
  }

  if (physicsSystem) {
    physicsSystem->update(deltaTime);
  }

  if (player) {
  }
}

void GameWorld::draw() const {
  for (const auto& obj : objects) {
    obj->draw();
  }
}

btDiscreteDynamicsWorld* GameWorld::getBulletWorld() const {
  return physicsSystem ? physicsSystem->dynamicsWorld : nullptr;
}

void GameWorld::initializeNavMesh() {
  // Create navigation mesh covering the game world
  Vector3 minBounds = {-30.0f, 0.0f, -30.0f};
  Vector3 maxBounds = {30.0f, 0.0f, 30.0f};
  navigationMesh = std::make_shared<NavMesh>(minBounds, maxBounds, 1.0f);

  navigationMesh->generateNavMesh();

  // First add shop buildings as obstacles
  auto shops = findObjectsOfType<Shop>();

  for (const auto& shop : shops) {
    Vector3 shopPos = shop->getPosition();
    Vector3 shopSize = shop->getSize();

    navigationMesh->addObstacle(shopPos, shopSize);
  }

  // Mark interiors and entrances inside the shops as walkable
  for (const auto& shop : shops) {
    Vector3 shopPos = shop->getPosition();
    Vector3 shopSize = shop->getSize();

    navigationMesh->defineShopInterior(shopPos, shopSize);

    Vector3 entrancePos = shop->getEntrancePosition();
    Vector3 entranceSize = {3.0f, 2.0f, 3.0f};

    navigationMesh->defineShopEntrance(entrancePos, entranceSize);
  }

  // Add all other static objects (e.g. shelves) as obstacles
  for (const auto& obj : objects) {
    if (!obj || !obj->getIsStatic()) continue;
    if (std::dynamic_pointer_cast<Shop>(obj)) continue;
    if (std::dynamic_pointer_cast<Floor>(obj))
      continue;  // Exclude Floor objects - they should be walkable ground

    Vector3 pos = obj->getPosition();
    Vector3 size = obj->getObstacleSize();
    navigationMesh->addObstacle(pos, size, obj->getObstacleType());
  }

  navigationMesh->rebuildConnections();

  NPC::setNavMesh(navigationMesh);
}

// Obstacle management implementations
void GameWorld::addObjectAsObstacle(std::shared_ptr<GameObject> object,
                                    const std::string& type) {
  if (!object) return;

  addObject(object);  // Add to world first

  if (navigationMesh) {
    Vector3 pos = object->getPosition();
    Vector3 size = object->getObstacleSize();
    std::string obstacleType = type.empty() ? object->getObstacleType() : type;

    navigationMesh->addObstacle(pos, size, obstacleType);
  }
}

void GameWorld::removeObjectObstacle(std::shared_ptr<GameObject> object) {
  if (!object || !navigationMesh) return;

  Vector3 pos = object->getPosition();
  Vector3 size = object->getObstacleSize();

  navigationMesh->removeObstacle(pos, size);
  removeObject(object);
}

void GameWorld::addObstacleAt(Vector3 position, Vector3 size,
                              const std::string& type) {
  if (navigationMesh) {
    navigationMesh->addObstacle(position, size, type);
  }
}

void GameWorld::addObjectAsObstacleDeferred(std::shared_ptr<GameObject> object,
                                            const std::string& type) {
  if (!object) return;

  addObject(object);  // Add to world first

  if (navigationMesh) {
    Vector3 pos = object->getPosition();
    Vector3 size = object->getObstacleSize();
    std::string obstacleType = type.empty() ? object->getObstacleType() : type;

    // Add obstacle but don't rebuild connections yet
    navigationMesh->markNodesInArea(pos, size, false,
                                    obstacleType == "shelf"  ? 0.4f
                                    : obstacleType == "wall" ? 0.2f
                                                             : 0.3f,
                                    obstacleType == "shelf");

  } else {
    // If navigation mesh is not initialized yet, we should still add the object
    // The navigation mesh will account for it when it's later initialized
  }
}

void GameWorld::finalizeObstacles() {
  if (navigationMesh) {
    navigationMesh->rebuildConnections();
  }
}