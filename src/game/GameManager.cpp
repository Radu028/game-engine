#include "game/GameManager.h"

#include <raylib.h>

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

  if (npcManager) {
    for (const auto& npc : npcManager->getActiveNPCs()) {
      if (!npc || !npc->getIsActive()) {
        continue;
      }
      drawNPCStatusBillboard(npc, camera);
    }
  }
}

void GameManager::drawNPCStatusBillboard(const std::shared_ptr<NPC>& npc,
                                         const Camera3D& camera) {
  constexpr float NPC_STATUS_BILLBOARD_WIDTH = 1.4f;
  constexpr float NPC_STATUS_VERTICAL_OFFSET = 1.8f;
  constexpr float NPC_STATUS_FONT_SIZE = 26.0f;
  constexpr int NPC_STATUS_PADDING = 6;

  if (!npc || !npc->getIsActive()) {
    return;
  }

  std::string stateText = npc->getCurrentStateName();
  if (stateText.empty()) {
    return;
  }

  Texture2D& texture = getOrCreateStatusTexture(stateText, NPC_STATUS_FONT_SIZE,
                                                NPC_STATUS_PADDING);

  Vector3 position = npc->getTorsoPosition();
  position.y += NPC_STATUS_VERTICAL_OFFSET;

  Vector2 size = {
      NPC_STATUS_BILLBOARD_WIDTH,
      NPC_STATUS_BILLBOARD_WIDTH * (static_cast<float>(texture.height) /
                                    static_cast<float>(texture.width))};
  Vector2 origin = {size.x * 0.5f, size.y};

  Rectangle source = {0.0f, 0.0f, static_cast<float>(texture.width),
                      static_cast<float>(texture.height)};

  DrawBillboardPro(camera, texture, source, position, Vector3{0.0f, 1.0f, 0.0f},
                   size, origin, 0.0f, WHITE);
}

Texture2D& GameManager::getOrCreateStatusTexture(const std::string& stateText,
                                                 float fontSize, int padding) {
  auto it = npcStatusTextures.find(stateText);
  if (it != npcStatusTextures.end()) {
    return it->second;
  }

  const Font& font = GetFontDefault();
  Vector2 measured = MeasureTextEx(font, stateText.c_str(), fontSize, 1);

  int width = static_cast<int>(measured.x * 1.1f) + padding * 2;
  int height = static_cast<int>(measured.y * 1.1f) + padding * 2;

  Image image = GenImageColor(width, height, BLANK);
  Color bgColor = {0, 0, 0, 160};
  ImageDrawRectangle(&image, 0, 0, width, height, bgColor);

  Vector2 textPos = {static_cast<float>(padding), static_cast<float>(padding)};
  ImageDrawTextEx(&image, font, stateText.c_str(), textPos, fontSize, 1, WHITE);

  Texture2D texture = LoadTextureFromImage(image);
  UnloadImage(image);

  auto inserted = npcStatusTextures.emplace(stateText, texture);
  return inserted.first->second;
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
