#include "shop/Fruit.h"

Color Fruit::getFruitColor(FruitType type) {
  switch (type) {
    case FruitType::APPLE:
      return RED;
    case FruitType::FRUIT_ORANGE:
      return {255, 165, 0, 255};  // Orange color
    case FruitType::BANANA:
      return YELLOW;
    case FruitType::GRAPE:
      return PURPLE;
    case FruitType::STRAWBERRY:
      return RED;
    default:
      return RED;
  }
}

float Fruit::getFruitRadius(FruitType type) {
  switch (type) {
    case FruitType::APPLE:
      return 0.15f;
    case FruitType::FRUIT_ORANGE:
      return 0.18f;
    case FruitType::BANANA:
      return 0.12f;
    case FruitType::GRAPE:
      return 0.08f;
    case FruitType::STRAWBERRY:
      return 0.10f;
    default:
      return 0.15f;
  }
}

float Fruit::getFruitPrice(FruitType type) {
  switch (type) {
    case FruitType::APPLE:
      return 1.5f;
    case FruitType::FRUIT_ORANGE:
      return 2.0f;
    case FruitType::BANANA:
      return 1.2f;
    case FruitType::GRAPE:
      return 2.5f;
    case FruitType::STRAWBERRY:
      return 3.0f;
    default:
      return 1.0f;
  }
}

std::string Fruit::getFruitName(FruitType type) {
  switch (type) {
    case FruitType::APPLE:
      return "Apple";
    case FruitType::FRUIT_ORANGE:
      return "Orange";
    case FruitType::BANANA:
      return "Banana";
    case FruitType::GRAPE:
      return "Grape";
    case FruitType::STRAWBERRY:
      return "Strawberry";
    default:
      return "Unknown";
  }
}

Fruit::Fruit(Vector3 position, FruitType fruitType)
    : Sphere(position, getFruitRadius(fruitType), getFruitColor(fruitType),
             true),
      type(fruitType),
      price(getFruitPrice(fruitType)),
      isPicked(false),
      name(getFruitName(fruitType)) {}

// Fruit interaction methods
void Fruit::pickFruit() { isPicked = true; }

void Fruit::resetFruit() { isPicked = false; }

void Fruit::interact() {
  if (!isPicked) {
    pickFruit();
  }
}

// Clone method for polymorphism
std::unique_ptr<GameObject> Fruit::clone() const {
  return std::make_unique<Fruit>(position, type);
}
