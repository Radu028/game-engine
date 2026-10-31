#pragma once

#include <memory>
#include <vector>

#include "objects/Wall.h"
#include "raylib.h"
#include "shop/Shelf.h"

// Shop building with walls, shelves, and entrance - Roblox style
class Shop {
 private:
  Vector3 position;
  Vector3 size;
  float entranceWidth;  // Width of the entrance gap

  std::vector<std::shared_ptr<Wall>> walls;
  std::vector<std::shared_ptr<Shelf>> shelves;

 public:
  Shop(Vector3 position, Vector3 size = {14.0f, 4.0f, 10.0f});
  ~Shop() = default;

  // Shop setup
  void buildWalls();
  void createShelves();
  void stockShelves();
  void createFloor();

  // Access methods
  const std::vector<std::shared_ptr<Shelf>>& getShelves() const {
    return shelves;
  }
  const std::vector<std::shared_ptr<Wall>>& getWalls() const { return walls; }

  // Position methods
  Vector3 getPosition() const { return position; }
  Vector3 getSize() const { return size; }
  Vector3 getRandomInteriorPosition() const;
  Vector3 getEntrancePosition() const;
  Vector3 getExitPosition() const;
  float getEntranceWidth() const { return entranceWidth; }

  // Bounds checking
  bool isInsideShop(Vector3 pos) const;
  bool isNearEntrance(Vector3 pos, float threshold = 3.0f) const;

  // NavMesh helpers
  void ensureEntranceWalkable();

  // Shop management
  void restockAllShelves();
  int getTotalFruitCount() const;
  bool isEmpty() const;
  std::shared_ptr<Fruit> findNearestFruit(Vector3 position);

  void interact();
  void update(float deltaTime);

 private:
  void addWall(Vector3 position, Vector3 size, Color color = DARKBROWN);
};
