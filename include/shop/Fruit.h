#pragma once

#include <string>

#include "objects/Sphere.h"
#include "raylib.h"

// Enum for different fruit types
enum class FruitType {
  APPLE,
  FRUIT_ORANGE,  // Renamed to avoid conflict with raylib's ORANGE macro
  BANANA,
  GRAPE,
  STRAWBERRY
};

// Fruit class representing individual fruits in the shop
class Fruit : public Sphere {
 private:
  FruitType type;
  float price;
  bool isPicked;
  std::string name;

 public:
  Fruit(Vector3 position, FruitType fruitType);
  ~Fruit() override = default;

  FruitType getType() const { return type; }
  float getPrice() const { return price; }
  bool getIsPicked() const { return isPicked; }
  const std::string& getName() const { return name; }

  // Fruits should not affect navigation/pathfinding.
  std::string getObstacleType() const override { return "pickup"; }

  void draw() const override;
  void pickFruit();
  void resetFruit();

  void interact() override;

  static Color getFruitColor(FruitType type);
  static float getFruitRadius(FruitType type);
  static float getFruitPrice(FruitType type);
  static std::string getFruitName(FruitType type);

  std::unique_ptr<GameObject> clone() const override;
};
