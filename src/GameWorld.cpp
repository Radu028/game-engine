#include "GameWorld.h"

#include "Player.h"

GameWorld* GameWorld::instance = nullptr;

GameWorld* GameWorld::getInstance(Player* player) {
  if (instance == nullptr) {
    instance = new GameWorld(player);
  }
  return instance;
}

GameWorld::GameWorld(Player* player) : player(player) {}

void GameWorld::addObject(std::shared_ptr<CubeObject> object) {
  objects.push_back(object);
}

void GameWorld::update(float deltaTime) {
  this->player->update(deltaTime);

  for (auto& obj : objects) {
    obj->update(deltaTime);
  }

  checkCollisions();
}

void GameWorld::draw() const {
  for (const auto& obj : objects) {
    obj->draw();
  }
}

void GameWorld::checkCollisions() {
  for (size_t i = 0; i < this->objects.size(); i++) {
    Vector3 prevObjectPosition = this->objects[i]->getPosition();

    for (size_t j = 0; j < this->objects.size(); j++) {
      if (i != j && this->objects[i]->checkCollision(*objects[j])) {
        this->objects[i]->setPosition(prevObjectPosition);
        // this->objects[i]->handleCollision(*this->objects[j]);
        break;
      }
    }
  }

  if (player) {
    Vector3 prevPlayerPosition = this->player->getPosition();

    // for (const auto& obj : objects) {
    //   if (this->player->checkCollision(*obj)) {
    //     this->player->setPosition(prevPlayerPosition);
    //     break;
    //   }
    // }
    if (this->player->checkCollisionWithWorld()) {
      this->player->setPosition(prevPlayerPosition);
    }
  }
}