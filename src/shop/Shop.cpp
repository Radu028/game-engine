#include "shop/Shop.h"

#include <random>

#include "GameWorld.h"
#include "raymath.h"

Shop::Shop(Vector3 position, Vector3 size) : position(position), size(size) {
  buildWalls();
  createShelves();
  stockShelves();
}

void Shop::buildWalls() {
  float wallThickness = 0.3f;
  float wallHeight = size.y;

  // Front wall
  addWall({position.x, position.y, position.z + size.z / 2.0f},
          {size.x, wallHeight, wallThickness});

  // Back wall
  addWall({position.x, position.y, position.z - size.z / 2.0f},
          {size.x, wallHeight, wallThickness});

  // Left wall
  addWall({position.x - size.x / 2.0f, position.y, position.z},
          {wallThickness, wallHeight, size.z});

  // Right wall
  addWall({position.x + size.x / 2.0f, position.y, position.z},
          {wallThickness, wallHeight, size.z});

  addWall({position.x, position.y + size.y / 2.0f, position.z},
          {size.x, wallThickness, size.z});
}

void Shop::createShelves() {
  float shelfHeight = 1.0f;
  float navigationMargin = 1.5f;

  for (int i = 0; i < 2; ++i) {
    Vector3 shelfPos = {position.x - size.x / 2.0f + navigationMargin,
                        position.y - size.y / 2.0f + shelfHeight,
                        position.z - size.z / 4.0f + i * size.z / 2.0f};
    auto shelf = std::make_shared<Shelf>(shelfPos);
    shelves.push_back(shelf);
  }

  for (int i = 0; i < 2; ++i) {
    Vector3 shelfPos = {position.x + size.x / 2.0f - navigationMargin,
                        position.y - size.y / 2.0f + shelfHeight,
                        position.z - size.z / 4.0f + i * size.z / 2.0f};
    auto shelf = std::make_shared<Shelf>(shelfPos);
    shelves.push_back(shelf);
  }

  Vector3 centerShelfPos = {
      position.x, position.y - size.y / 2.0f + shelfHeight, position.z};
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

Vector3 Shop::getRandomInteriorPosition() const {
  static std::random_device rd;
  static std::mt19937 gen(rd());

  float safeMargin = 1.0f;

  std::uniform_real_distribution<float> xDist(
      position.x - size.x / 2.0f + safeMargin,
      position.x + size.x / 2.0f - safeMargin);
  std::uniform_real_distribution<float> zDist(
      position.z - size.z / 2.0f + safeMargin,
      position.z + size.z / 2.0f - safeMargin);

  return {xDist(gen), position.y - size.y / 2.0f + 1.0f, zDist(gen)};
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
  for (auto& shelf : shelves) {
    shelf->update(deltaTime);
  }
}

void Shop::addWall(Vector3 position, Vector3 size, Color color) {
  auto wall = std::make_shared<Wall>(position, size, color);
  walls.push_back(wall);
}

Vector3 Shop::calculateInteriorBounds() const {
  return {
      size.x - 2.0f,  // Leave space for walls
      size.y - 1.0f,  // Leave space for floor/ceiling
      size.z - 2.0f   // Leave space for walls
  };
}

void Shop::finalizeNavMesh() {
  if (GameWorld* world = GameWorld::getInstance(nullptr)) {
    world->finalizeObstacles();
  }
}
