#pragma once

#include <memory>
#include <vector>

#include "objects/Wall.h"
#include "raylib.h"
#include "shop/Shelf.h"

// Shop building with walls and shelves
class Shop {
 private:
  Vector3 position;
  Vector3 size;

  std::vector<std::shared_ptr<Wall>> walls;
  std::vector<std::shared_ptr<Shelf>> shelves;

 public:
  Shop(Vector3 position, Vector3 size = {12.0f, 4.0f, 8.0f});
  ~Shop() = default;

  // Shop setup
  void buildWalls();
  void createShelves();
  void stockShelves();

  // Access methods
  const std::vector<std::shared_ptr<Shelf>>& getShelves() const {
    return shelves;
  }
  const std::vector<std::shared_ptr<Wall>>& getWalls() const { return walls; }

  // Position checking
  bool isInsideShop(Vector3 position) const;
  Vector3 getRandomInteriorPosition() const;

  // Shop management
  void restockAllShelves();
  int getTotalFruitCount() const;
  bool isEmpty() const;
  std::shared_ptr<Fruit> findNearestFruit(Vector3 position);

  void interact();
  void update(float deltaTime);
  std::unique_ptr<GameObject> clone() const;

 private:
  void addWall(Vector3 position, Vector3 size, Color color = DARKBROWN);
  Vector3 calculateInteriorBounds() const;
  void finalizeNavMesh();
};
