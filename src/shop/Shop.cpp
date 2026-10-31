#include "shop/Shop.h"

#include <algorithm>
#include <limits>
#include <random>

#include "GameWorld.h"
#include "ai/NavMesh.h"
#include "objects/Floor.h"
#include "raymath.h"

namespace {
// Roblox-style colors
const Color WALL_COLOR = {139, 90, 43, 255};       // Brown wood
const Color FLOOR_COLOR = {160, 140, 120, 255};    // Light wood floor
const Color CEILING_COLOR = {180, 160, 140, 255};  // Lighter ceiling
}  // namespace

Shop::Shop(Vector3 position, Vector3 size)
    : position(position), size(size), entranceWidth(6.0f) {
  entranceWidth =
      std::clamp(entranceWidth, 2.0f, std::max(2.0f, size.x - 1.0f));
  createFloor();
  buildWalls();
  createShelves();
  stockShelves();
}

void Shop::createFloor() {
  // Create shop floor
  if (GameWorld* world = GameWorld::getInstance()) {
    Vector3 floorPos = {position.x, position.y - size.y / 2.0f - 0.05f,
                        position.z};
    Vector3 floorSize = {size.x + 0.2f, 0.1f, size.z + 0.2f};
    auto floor =
        std::make_shared<Floor>(floorPos, floorSize, FLOOR_COLOR, true);
    world->addObject(floor);
  }
}

void Shop::buildWalls() {
  float wallThickness = 0.25f;  // Thinner walls
  float wallHeight = size.y;

  // Front wall (south side - with wide entrance gap in the middle)
  float frontZ = position.z + size.z / 2.0f;
  float sideWallWidth = std::max(0.0f, (size.x - entranceWidth) / 2.0f);

  if (sideWallWidth > 0.01f) {
    // Left part of front wall - positioned away from entrance
    float leftWallX = position.x - size.x / 2.0f + sideWallWidth / 2.0f;
    addWall({leftWallX, position.y, frontZ},
            {sideWallWidth, wallHeight, wallThickness}, WALL_COLOR);

    // Right part of front wall - positioned away from entrance
    float rightWallX = position.x + size.x / 2.0f - sideWallWidth / 2.0f;
    addWall({rightWallX, position.y, frontZ},
            {sideWallWidth, wallHeight, wallThickness}, WALL_COLOR);
  }

  // Back wall (north side - solid)
  addWall({position.x, position.y, position.z - size.z / 2.0f},
          {size.x, wallHeight, wallThickness}, WALL_COLOR);

  // Left wall (west side - solid)
  addWall({position.x - size.x / 2.0f, position.y, position.z},
          {wallThickness, wallHeight, size.z - wallThickness}, WALL_COLOR);

  // Right wall (east side - solid)
  addWall({position.x + size.x / 2.0f, position.y, position.z},
          {wallThickness, wallHeight, size.z - wallThickness}, WALL_COLOR);

  // Ceiling
  addWall({position.x, position.y + size.y / 2.0f, position.z},
          {size.x, wallThickness, size.z}, CEILING_COLOR);
}

void Shop::createShelves() {
  float shelfHeight = 1.0f;
  float navigationMargin = 2.0f;  // More space for navigation
  float shelfSpacing = 3.0f;

  // Left row of shelves (2 shelves along the left wall)
  for (int i = 0; i < 2; ++i) {
    Vector3 shelfPos = {position.x - size.x / 2.0f + navigationMargin,
                        position.y - size.y / 2.0f + shelfHeight,
                        position.z - size.z / 4.0f + i * shelfSpacing};
    auto shelf = std::make_shared<Shelf>(shelfPos);
    shelves.push_back(shelf);
  }

  // Right row of shelves (2 shelves along the right wall)
  for (int i = 0; i < 2; ++i) {
    Vector3 shelfPos = {position.x + size.x / 2.0f - navigationMargin,
                        position.y - size.y / 2.0f + shelfHeight,
                        position.z - size.z / 4.0f + i * shelfSpacing};
    auto shelf = std::make_shared<Shelf>(shelfPos);
    shelves.push_back(shelf);
  }

  // Center shelf (positioned toward the back to leave entrance area clear)
  Vector3 centerShelfPos = {position.x,
                            position.y - size.y / 2.0f + shelfHeight,
                            position.z - size.z / 4.0f};
  auto centerShelf = std::make_shared<Shelf>(centerShelfPos);
  shelves.push_back(centerShelf);

  if (GameWorld* world = GameWorld::getInstance()) {
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

  float safeMargin = 2.0f;

  std::uniform_real_distribution<float> xDist(
      position.x - size.x / 2.0f + safeMargin,
      position.x + size.x / 2.0f - safeMargin);
  // Avoid the entrance area (front part of shop)
  std::uniform_real_distribution<float> zDist(
      position.z - size.z / 2.0f + safeMargin,
      position.z + size.z / 4.0f);  // Stay away from entrance

  return {xDist(gen), position.y - size.y / 2.0f + 1.0f, zDist(gen)};
}

Vector3 Shop::getEntrancePosition() const {
  // Center of the entrance, just inside the shop
  return {position.x, position.y - size.y / 2.0f + 1.0f,
          position.z + size.z / 2.0f - 1.0f};
}

Vector3 Shop::getExitPosition() const {
  // Position just outside the entrance
  return {position.x, position.y - size.y / 2.0f + 1.0f,
          position.z + size.z / 2.0f + 3.0f};
}

bool Shop::isInsideShop(Vector3 pos) const {
  float halfX = size.x / 2.0f;
  float halfZ = size.z / 2.0f;

  return pos.x >= position.x - halfX && pos.x <= position.x + halfX &&
         pos.z >= position.z - halfZ && pos.z <= position.z + halfZ;
}

bool Shop::isNearEntrance(Vector3 pos, float threshold) const {
  Vector3 entrancePos = getEntrancePosition();
  float dx = pos.x - entrancePos.x;
  float dz = pos.z - entrancePos.z;
  return (dx * dx + dz * dz) < threshold * threshold;
}

void Shop::ensureEntranceWalkable() {
  if (GameWorld* world = GameWorld::getInstance()) {
    if (auto navMesh = world->getNavMesh()) {
      float frontZ = position.z + size.z / 2.0f;

      // Clear a wide corridor through the entrance
      Vector3 entranceCenter = {position.x, 0.5f, frontZ};
      Vector3 entranceSize = {entranceWidth, 1.0f, 2.0f};
      navMesh->ensureAreaWalkable(entranceCenter, entranceSize);

      // Clear the approach path outside the shop (multiple segments)
      for (float z = frontZ + 1.0f; z <= frontZ + 10.0f; z += 2.0f) {
        Vector3 pathCenter = {position.x, 0.5f, z};
        Vector3 pathSize = {entranceWidth + 2.0f, 1.0f, 2.5f};
        navMesh->ensureAreaWalkable(pathCenter, pathSize);
      }

      // Clear the area just inside the shop
      Vector3 insideCenter = {position.x, 0.5f, frontZ - 2.0f};
      Vector3 insideSize = {entranceWidth + 1.0f, 1.0f, 4.0f};
      navMesh->ensureAreaWalkable(insideCenter, insideSize);

      navMesh->rebuildConnections();
    }
  }
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

std::shared_ptr<Fruit> Shop::findNearestFruit(Vector3 pos) {
  std::shared_ptr<Fruit> nearest = nullptr;
  float minDistance = std::numeric_limits<float>::max();

  for (auto& shelf : shelves) {
    auto fruit = shelf->getNearestFruit(pos);
    if (fruit) {
      float distance = Vector3Distance(pos, fruit->getPosition());
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

void Shop::addWall(Vector3 wallPos, Vector3 wallSize, Color color) {
  auto wall = std::make_shared<Wall>(wallPos, wallSize, color);
  walls.push_back(wall);

  // Add wall to GameWorld for physics and collision
  if (GameWorld* world = GameWorld::getInstance()) {
    world->addObjectAsObstacleDeferred(wall);
  }
}
