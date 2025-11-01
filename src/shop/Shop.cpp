#include "shop/Shop.h"

#include <random>

#include "GameWorld.h"
#include "raymath.h"

Shop::Shop(Vector3 position, Vector3 size)
    : CubeObject(position, size, BEIGE, false, "", false, false, false),
      entranceSize({4.0f, 3.0f, 1.0f}) {
  entrancePosition = {position.x, position.y - size.y / 2.0f + 0.5f,
                      position.z + size.z / 2.0f - 0.5f};

  interiorBounds = calculateInteriorBounds();

  buildWalls();
  createShelves();
  stockShelves();
}

void Shop::buildWalls() {
  Vector3 shopPos = getPosition();
  Vector3 shopSize = getSize();

  float wallThickness = 0.3f;
  float wallHeight = shopSize.y;

  float entranceWidth = entranceSize.x;
  float frontWallWidth = (shopSize.x - entranceWidth) / 2.0f;

  // Front wall (left side of entrance)
  addWall({shopPos.x - shopSize.x / 2.0f + frontWallWidth / 2.0f, shopPos.y,
           shopPos.z + shopSize.z / 2.0f},
          {frontWallWidth, wallHeight, wallThickness});

  // Front wall (right side of entrance)
  addWall({shopPos.x + shopSize.x / 2.0f - frontWallWidth / 2.0f, shopPos.y,
           shopPos.z + shopSize.z / 2.0f},
          {frontWallWidth, wallHeight, wallThickness});

  // Back wall
  addWall({shopPos.x, shopPos.y, shopPos.z - shopSize.z / 2.0f},
          {shopSize.x, wallHeight, wallThickness});

  // Left wall
  addWall({shopPos.x - shopSize.x / 2.0f, shopPos.y, shopPos.z},
          {wallThickness, wallHeight, shopSize.z});

  // Right wall
  addWall({shopPos.x + shopSize.x / 2.0f, shopPos.y, shopPos.z},
          {wallThickness, wallHeight, shopSize.z});

  addWall({shopPos.x, shopPos.y + shopSize.y / 2.0f, shopPos.z},
          {shopSize.x, wallThickness, shopSize.z});
}

void Shop::createShelves() {
  Vector3 shopPos = getPosition();
  Vector3 shopSize = getSize();

  float shelfHeight = 1.0f;
  float navigationMargin = 1.5f;

  for (int i = 0; i < 2; ++i) {
    Vector3 shelfPos = {shopPos.x - shopSize.x / 2.0f + navigationMargin,
                        shopPos.y - shopSize.y / 2.0f + shelfHeight,
                        shopPos.z - shopSize.z / 4.0f + i * shopSize.z / 2.0f};
    auto shelf = std::make_shared<Shelf>(shelfPos);
    shelves.push_back(shelf);
  }

  for (int i = 0; i < 2; ++i) {
    Vector3 shelfPos = {shopPos.x + shopSize.x / 2.0f - navigationMargin,
                        shopPos.y - shopSize.y / 2.0f + shelfHeight,
                        shopPos.z - shopSize.z / 4.0f + i * shopSize.z / 2.0f};
    auto shelf = std::make_shared<Shelf>(shelfPos);
    shelves.push_back(shelf);
  }

  Vector3 centerShelfPos = {
      shopPos.x, shopPos.y - shopSize.y / 2.0f + shelfHeight, shopPos.z};
  auto centerShelf = std::make_shared<Shelf>(centerShelfPos);
  shelves.push_back(centerShelf);

  if (GameWorld* world = GameWorld::getInstance(nullptr)) {
    for (auto& shelf : shelves) {
      world->addObjectAsObstacleDeferred(shelf);
    }
  }
}

void Shop::stockShelves() {
  for (auto& shelf : shelves) {
    shelf->restockAllFruits();
  }
}

bool Shop::isInsideShop(Vector3 position) const {
  Vector3 shopPos = getPosition();
  Vector3 shopSize = getSize();

  float wallThickness = 0.3f;
  float interiorMargin = wallThickness + 0.1f;

  return (position.x >= shopPos.x - shopSize.x / 2.0f + interiorMargin &&
          position.x <= shopPos.x + shopSize.x / 2.0f - interiorMargin &&
          position.z >= shopPos.z - shopSize.z / 2.0f + interiorMargin &&
          position.z <= shopPos.z + shopSize.z / 2.0f - interiorMargin &&
          position.y >= shopPos.y - shopSize.y / 2.0f &&
          position.y <= shopPos.y + shopSize.y / 2.0f);
}

bool Shop::isNearEntrance(Vector3 position, float threshold) const {
  Vector3 entrancePos = getEntrancePosition();
  float distance = Vector3Distance(position, entrancePos);

  float entranceHalfWidth = entranceSize.x / 2.0f;
  float entranceDepth = entranceSize.z + 2.0f;

  bool withinEntranceBounds =
      (position.x >= entrancePos.x - entranceHalfWidth - 1.0f &&
       position.x <= entrancePos.x + entranceHalfWidth + 1.0f &&
       position.z >= entrancePos.z - entranceDepth &&
       position.z <= entrancePos.z + entranceDepth);

  return distance <= threshold || withinEntranceBounds;
}

Vector3 Shop::getRandomInteriorPosition() const {
  static std::random_device rd;
  static std::mt19937 gen(rd());

  Vector3 shopPos = getPosition();
  Vector3 shopSize = getSize();

  // Create smaller safe margin to allow NPCs closer to shelves
  float safeMargin = 1.0f;  // Reduced from 2.0f to get closer to shelves

  std::uniform_real_distribution<float> xDist(
      shopPos.x - shopSize.x / 2.0f + safeMargin,
      shopPos.x + shopSize.x / 2.0f - safeMargin);
  std::uniform_real_distribution<float> zDist(
      shopPos.z - shopSize.z / 2.0f + safeMargin,
      shopPos.z + shopSize.z / 2.0f - safeMargin);

  return {xDist(gen), shopPos.y - shopSize.y / 2.0f + 1.0f, zDist(gen)};
}

void Shop::restockAllShelves() {
  for (auto& shelf : shelves) {
    shelf->restockAllFruits();
  }
}

int Shop::getTotalFruitCount() const {
  int total = 0;
  for (const auto& shelf : shelves) {
    total += shelf->getFruitCount();
  }
  return total;
}

bool Shop::isEmpty() const { return getTotalFruitCount() == 0; }

std::shared_ptr<Fruit> Shop::findNearestFruit(Vector3 position) {
  std::shared_ptr<Fruit> nearest = nullptr;
  float minDistance = std::numeric_limits<float>::max();

  for (auto& shelf : shelves) {
    auto fruit = shelf->getNearestFruit(position);
    if (fruit) {
      float distance = Vector3Distance(position, fruit->getPosition());
      if (distance < minDistance) {
        minDistance = distance;
        nearest = fruit;
      }
    }
  }

  return nearest;
}

void Shop::interact() { restockAllShelves(); }

void Shop::update(float deltaTime) {
  CubeObject::update(deltaTime);

  for (auto& shelf : shelves) {
    shelf->update(deltaTime);
  }
}

std::unique_ptr<GameObject> Shop::clone() const {
  return std::make_unique<Shop>(*this);
}

void Shop::addWall(Vector3 position, Vector3 size, Color color) {
  auto wall = std::make_shared<CubeObject>(position, size, color, true, "",
                                           false, true, true);
  walls.push_back(wall);

  if (GameWorld* world = GameWorld::getInstance(nullptr)) {
    world->addObjectAsObstacleDeferred(wall);
  }
}

Vector3 Shop::calculateInteriorBounds() const {
  Vector3 shopSize = getSize();
  return {
      shopSize.x - 2.0f,  // Leave space for walls
      shopSize.y - 1.0f,  // Leave space for floor/ceiling
      shopSize.z - 2.0f   // Leave space for walls
  };
}

void Shop::finalizeNavMesh() {
  if (GameWorld* world = GameWorld::getInstance(nullptr)) {
    world->finalizeObstacles();
  }
}
