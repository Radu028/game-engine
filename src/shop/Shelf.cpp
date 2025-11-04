#include "shop/Shelf.h"

#include <algorithm>

#include "GameWorld.h"
#include "raymath.h"

Shelf::Shelf(Vector3 position, Vector3 size, Color color)
    : CubeObject(position, size, color, true, "", false, true, true),
      shelfPosition(position),
      maxFruits(8),
      fruitSpacing(0.3f) {}

void Shelf::addFruit(std::shared_ptr<Fruit> fruit) {
  if (fruits.size() < maxFruits && fruit && !fruit->getIsPicked()) {
    fruits.push_back(fruit);
    arrangeFruits();
  }
}

void Shelf::removeFruit(std::shared_ptr<Fruit> fruit) {
  auto it = std::find(fruits.begin(), fruits.end(), fruit);
  if (it != fruits.end()) {
    fruits.erase(it);
    arrangeFruits();
  }
}

bool Shelf::hasFruit(FruitType type) const {
  return std::any_of(fruits.begin(), fruits.end(),
                     [type](const std::shared_ptr<Fruit>& fruit) {
                       return fruit && fruit->getType() == type &&
                              !fruit->getIsPicked();
                     });
}

std::shared_ptr<Fruit> Shelf::getFruit(FruitType type) {
  auto it = std::find_if(fruits.begin(), fruits.end(),
                         [type](const std::shared_ptr<Fruit>& fruit) {
                           return fruit && fruit->getType() == type &&
                                  !fruit->getIsPicked();
                         });

  return (it != fruits.end()) ? *it : nullptr;
}

std::shared_ptr<Fruit> Shelf::getNearestFruit(Vector3 position) {
  std::shared_ptr<Fruit> nearest = nullptr;
  float minDistance = std::numeric_limits<float>::max();

  for (auto& fruit : fruits) {
    if (fruit && !fruit->getIsPicked()) {
      float distance = Vector3Distance(position, fruit->getPosition());
      if (distance < minDistance) {
        minDistance = distance;
        nearest = fruit;
      }
    }
  }

  return nearest;
}

void Shelf::restockShelf(FruitType type, int quantity) {
  int currentCount = getFruitCount(type);
  int needed = quantity - currentCount;

  for (int i = 0; i < needed && fruits.size() < maxFruits; ++i) {
    Vector3 fruitPos = calculateFruitPosition(fruits.size());
    auto newFruit = std::make_shared<Fruit>(fruitPos, type);
    addFruit(newFruit);

    if (GameWorld* world = GameWorld::getInstance()) {
      world->addObject(newFruit);
    }
  }
}

void Shelf::restockAllFruits() {
  restockShelf(FruitType::APPLE, 2);
  restockShelf(FruitType::FRUIT_ORANGE, 2);
  restockShelf(FruitType::BANANA, 2);
  restockShelf(FruitType::GRAPE, 1);
  restockShelf(FruitType::STRAWBERRY, 1);
}

int Shelf::getFruitCount() const {
  return std::count_if(fruits.begin(), fruits.end(),
                       [](const std::shared_ptr<Fruit>& fruit) {
                         return fruit && !fruit->getIsPicked();
                       });
}

int Shelf::getFruitCount(FruitType type) const {
  return std::count_if(fruits.begin(), fruits.end(),
                       [type](const std::shared_ptr<Fruit>& fruit) {
                         return fruit && fruit->getType() == type &&
                                !fruit->getIsPicked();
                       });
}

bool Shelf::isEmpty() const { return getFruitCount() == 0; }

void Shelf::interact() { arrangeFruits(); }

void Shelf::update(float deltaTime) {
  CubeObject::update(deltaTime);

  fruits.erase(std::remove_if(fruits.begin(), fruits.end(),
                              [](const std::shared_ptr<Fruit>& fruit) {
                                return fruit && fruit->getIsPicked();
                              }),
               fruits.end());
}

std::unique_ptr<GameObject> Shelf::clone() const {
  return std::make_unique<Shelf>(*this);
}

Vector3 Shelf::calculateFruitPosition(int index) const {
  Vector3 shelfSize = getSize();
  float shelfTop = shelfPosition.y + shelfSize.y / 2.0f;

  // Arrange fruits in rows
  int fruitsPerRow = 4;
  int row = index / fruitsPerRow;
  int col = index % fruitsPerRow;

  float startX = shelfPosition.x - (shelfSize.x / 2.0f) + 0.2f;
  float startZ = shelfPosition.z - (shelfSize.z / 2.0f) + 0.2f;

  Vector3 fruitPos = {
      startX + col * fruitSpacing,
      shelfTop + 0.15f + row * 0.2f,  // Stack fruits vertically if needed
      startZ + row * 0.2f};

  return fruitPos;
}

void Shelf::arrangeFruits() {
  for (size_t i = 0; i < fruits.size(); ++i) {
    if (fruits[i] && !fruits[i]->getIsPicked()) {
      Vector3 newPos = calculateFruitPosition(i);
      fruits[i]->setPosition(newPos);
    }
  }
}
