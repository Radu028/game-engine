#include "game/GameManager.h"

#include "GameWorld.h"
#include "ai/NPCManager.h"

GameManager* GameManager::instance = nullptr;

GameManager::GameManager()
    : npcManager(nullptr),
      chatSystem(std::make_unique<NPCChatSystem>()),
      initialStockPerFruit(10),
      totalFruitsInStock(0),
      gameTimer(0.0f),
      fruitsPickedByNPCs(0),
      npcsSoFar(0) {
  initializeShop();

  if (GameWorld* world = GameWorld::getInstance()) {
    world->initializeNavMesh();
    world->finalizeObstacles();
  }

  npcManager = NPCManager::getInstance();
  if (npcManager && shop) {
    npcManager->setShop(shop);
    npcManager->setChatSystem(chatSystem.get());
    npcManager->setSpawnInterval(2.0f, 4.0f);
    npcManager->setMaxActiveNPCs(5);
  }
}

GameManager::~GameManager() {}

GameManager* GameManager::getInstance() {
  if (instance == nullptr) {
    instance = new GameManager();
  }
  return instance;
}

void GameManager::destroyInstance() {
  delete instance;
  instance = nullptr;
}

void GameManager::update(float deltaTime) {
  gameTimer += deltaTime;

  if (npcManager) {
    npcManager->update(deltaTime);
  }

  if (chatSystem) {
    chatSystem->update(deltaTime);
  }

  if (shop) {
    shop->update(deltaTime);
  }

  updateGameLogic(deltaTime);
}

void GameManager::render(Camera3D camera) {
  if (chatSystem) {
    chatSystem->drawAllMessages(camera);
  }

  if (GameWorld* world = GameWorld::getInstance()) {
    if (auto navMesh = world->getNavMesh()) {
      navMesh->debugDraw();
      if (shop) {
        navMesh->debugDrawEntranceNodes(shop->getEntrancePosition(),
                                        shop->getEntranceSize());
      }
    }
  }
}

int GameManager::getRemainingFruits() const {
  if (!shop) {
    return 0;
  }

  return shop->getTotalFruitCount();
}

int GameManager::getNPCCount() const {
  if (!npcManager) {
    return 0;
  }

  return npcManager->getActiveNPCCount();
}

void GameManager::onFruitPicked() { fruitsPickedByNPCs++; }

void GameManager::onNPCSpawned() { npcsSoFar++; }

void GameManager::setShop(std::shared_ptr<Shop> newShop) {
  shop = newShop;

  if (npcManager && shop) {
    npcManager->setShop(shop);
  }
}

void GameManager::updateGameLogic(float deltaTime) {
  // Update game-specific logic here

  // Sync statistics with NPC manager
  if (npcManager) {
    fruitsPickedByNPCs = npcManager->getTotalFruitsPicked();
    npcsSoFar = npcManager->getTotalNPCsSpawned();
  }
}

void GameManager::initializeShop() {
  // Create shop at the center of the world
  Vector3 shopPosition = {0.0f, 2.0f, -10.0f};
  shop = std::make_shared<Shop>(shopPosition);

  // Add shop to game world
  if (GameWorld* world = GameWorld::getInstance(nullptr)) {
    world->addObject(shop);
  }

  // Count initial fruits
  totalFruitsInStock = shop->getTotalFruitCount();
}
