#include "GameWorld.h"

#include <algorithm>
#include <ctime>
#include <sstream>

#include "ai/NPC.h"
#include "ai/NavMesh.h"
#include "shop/Shop.h"
#include "systems/PhysicsSystem.h"

GameWorld *GameWorld::instance = nullptr;
size_t GameWorld::totalObjectsCreated = 0;
size_t GameWorld::totalObjectsDestroyed = 0;
std::unordered_map<std::string, size_t> GameWorld::objectTypeCount;
std::string GameWorld::creationTimestamp;

GameWorld *GameWorld::getInstance(GameObject *player) {
  if (instance == nullptr) {
    instance = new GameWorld(player);
  }
  return instance;
}

GameWorld::GameWorld(GameObject *player, const std::string &name)
    : player(player), worldName(name) {
  // Set creation timestamp
  auto now = std::time(nullptr);
  std::stringstream ss;
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

  // Automatically register object with NavMesh for professional pathfinding
  if (navigationMesh) {
    // Skip NPC bodies from being treated as obstacles when registering
    if (object->getObstacleType() != "npc") {
      navigationMesh->registerObject(object.get());
    }
  }
}

void GameWorld::removeObject(std::shared_ptr<GameObject> object) {
  if (!object) return;

  if (physicsSystem) {
    physicsSystem->removeObject(object.get());
  }

  // Automatically unregister object from NavMesh
  if (navigationMesh) {
    navigationMesh->unregisterObject(object.get());
  }

  objects.erase(std::remove_if(objects.begin(), objects.end(),
                               [&](const std::shared_ptr<GameObject> &obj) {
                                 return obj == object;
                               }),
                objects.end());
}

void GameWorld::update(float deltaTime) {
  for (auto &objSharedPtr : objects) {
    if (objSharedPtr.get() != player) {
      objSharedPtr->update(deltaTime);
    }
  }

  if (physicsSystem) {
    physicsSystem->update(deltaTime);
  }

  // Optimized NavMesh: only cleans up invalid objects, no position updates
  if (navigationMesh) {
    navigationMesh->updateAllRegisteredObjects();
  }

  if (player) {
  }
}

void GameWorld::draw() const {
  for (const auto &obj : objects) {
    obj->draw();
  }
}

btDiscreteDynamicsWorld *GameWorld::getBulletWorld() const {
  return physicsSystem ? physicsSystem->dynamicsWorld : nullptr;
}

void GameWorld::initializeNavMesh() {
  // Create navigation mesh covering the game world - balanced size for good
  // performance and coverage
  Vector3 minBounds = {-25.0f, 0.0f, -25.0f};
  Vector3 maxBounds = {25.0f, 0.0f, 25.0f};
  float groundLevel = 0.0f;  // Define ground level for blocking calculations

  navigationMesh = std::make_shared<NavMesh>(
      minBounds, maxBounds,
      1.2f,          // Balanced spacing: smaller than 1.5f but larger than 1.0f
      groundLevel);  // Set ground level for professional blocking calculations

  navigationMesh->generateNavMesh();

  // Register all existing objects with the new NavMesh
  // This calculates blocking ONCE for each static object
  for (const auto &object : objects) {
    if (object) {
      navigationMesh->registerObject(object.get());
    }
  }

  // Build connections ONCE at the end after all static obstacles are set
  navigationMesh->rebuildConnections();

  NPC::setNavMesh(navigationMesh);
}

// Obstacle management implementations
void GameWorld::addObjectAsObstacle(std::shared_ptr<GameObject> object,
                                    const std::string &type) {
  if (!object) return;

  addObject(object);  // Add to world first

  if (navigationMesh) {
    // Only mark nodes if the object actually blocks pathfinding
    if (navigationMesh->shouldObjectBlockPath(object.get())) {
      Vector3 pos = object->getPosition();
      Vector3 size = object->getObstacleSize();
      std::string obstacleType =
          type.empty() ? object->getObstacleType() : type;

      navigationMesh->addObstacle(pos, size, obstacleType);
    }
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
                              const std::string &type) {
  if (navigationMesh) {
    navigationMesh->addObstacle(position, size, type);
  }
}

void GameWorld::addObjectAsObstacleDeferred(std::shared_ptr<GameObject> object,
                                            const std::string &type) {
  if (!object) return;

  addObject(object);  // Add to world first

  if (navigationMesh) {
    // Only mark nodes if the object actually blocks pathfinding
    if (navigationMesh->shouldObjectBlockPath(object.get())) {
      Vector3 pos = object->getPosition();
      Vector3 size = object->getObstacleSize();
      std::string obstacleType =
          type.empty() ? object->getObstacleType() : type;

      // Add obstacle but don't rebuild connections yet
      navigationMesh->markNodesInArea(pos, size, false,
                                      obstacleType == "shelf"  ? 0.7f
                                      : obstacleType == "wall" ? 0.3f
                                                               : 0.5f);
    }
  } else {
  }
}

void GameWorld::finalizeObstacles() {
  if (navigationMesh) {
    navigationMesh->rebuildConnections();
  }
}