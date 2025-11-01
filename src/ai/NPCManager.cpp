#include "ai/NPCManager.h"

#include <algorithm>

#include "GameWorld.h"

NPCManager* NPCManager::instance = nullptr;

NPCManager::NPCManager()
    : spawnTimer(0.0f),
      spawnInterval(5.0f),
      minSpawnInterval(3.0f),
      maxSpawnInterval(8.0f),
      maxActiveNPCs(5),
      chatSystem(nullptr),
      totalNPCsSpawned(0),
      totalFruitsPicked(0),
      totalNPCsExited(0),
      gen(rd()) {}

NPCManager::~NPCManager() {
  // Clean up any remaining NPCs
  for (auto& npc : activeNPCs) {
    npc->removeObserver(this);
  }
  activeNPCs.clear();
}

NPCManager* NPCManager::getInstance() {
  if (instance == nullptr) {
    instance = new NPCManager();
  }
  return instance;
}

void NPCManager::destroyInstance() {
  delete instance;
  instance = nullptr;
}

void NPCManager::setShop(std::shared_ptr<Shop> targetShop) {
  shop = targetShop;
}

void NPCManager::update(float deltaTime) {
  updateSpawnTimer(deltaTime);

  for (auto& npc : activeNPCs) {
    npc->update(deltaTime);
  }

  removeInactiveNPCs();

  if (shouldSpawnNPC()) {
    spawnNPC();
  }
}

void NPCManager::spawnNPC() {
  if (!shop) {
    return;
  }

  auto newNPC = createNPC();
  if (newNPC) {
    if (chatSystem) {
      newNPC->setChatSystem(chatSystem);
    }

    activeNPCs.push_back(newNPC);
    newNPC->addObserver(this);

    if (GameWorld* world = GameWorld::getInstance(nullptr)) {
      world->addObject(newNPC);

      newNPC->setupPhysics(world->getDynamicsWorld());
    }

    totalNPCsSpawned++;

    std::uniform_real_distribution<float> intervalDist(minSpawnInterval,
                                                       maxSpawnInterval);
    spawnInterval = intervalDist(gen);
    spawnTimer = 0.0f;
  }
}

void NPCManager::removeInactiveNPCs() {
  auto it = std::remove_if(
      activeNPCs.begin(), activeNPCs.end(),
      [this](const std::shared_ptr<NPC>& npc) {
        if (!npc->getIsActive()) {
          npc->removeObserver(this);

          // Remove from game world
          if (GameWorld* world = GameWorld::getInstance(nullptr)) {
            npc->removeFromPhysics(world->getDynamicsWorld());
            world->removeObject(npc);
          }

          return true;
        }
        return false;
      });

  activeNPCs.erase(it, activeNPCs.end());
}

std::shared_ptr<NPC> NPCManager::createNPC() {
  if (!shop) {
    return nullptr;
  }

  Vector3 spawnPos = getRandomSpawnPosition();
  auto npc = std::make_shared<NPC>(spawnPos, shop);

  return npc;
}

void NPCManager::onNPCFruitPicked(NPC* npc, std::shared_ptr<Fruit> fruit) {
  totalFruitsPicked++;

  if (fruit) {
    totalFruitsPicked++;
  }
}

void NPCManager::onNPCExited(NPC* npc) { totalNPCsExited++; }

void NPCManager::onNPCEnteredShop(NPC* npc) {
  // Track NPC shop entry
}

void NPCManager::setSpawnInterval(float min, float max) {
  minSpawnInterval = min;
  maxSpawnInterval = max;

  // Update current interval if needed
  std::uniform_real_distribution<float> intervalDist(minSpawnInterval,
                                                     maxSpawnInterval);
  spawnInterval = intervalDist(gen);
}

Vector3 NPCManager::getRandomSpawnPosition() const {
  Vector3 shopPos = shop ? shop->getPosition() : Vector3{0, 0, 0};
  Vector3 entrancePos = shop ? shop->getEntrancePosition() : Vector3{0, 0, 0};

  std::uniform_real_distribution<float> xOffset(-3.0f, 8.0f);
  std::uniform_real_distribution<float> zOffset(6.0f, 12.0f);

  Vector3 spawnPos = {entrancePos.x + xOffset(gen), 0.5f,
                      entrancePos.z + zOffset(gen)};

  return spawnPos;
}

bool NPCManager::shouldSpawnNPC() const {
  return spawnTimer >= spawnInterval && shop != nullptr &&
         static_cast<int>(activeNPCs.size()) < maxActiveNPCs;
}

void NPCManager::updateSpawnTimer(float deltaTime) { spawnTimer += deltaTime; }

void NPCManager::cleanupNPCs() {
  for (auto& npc : activeNPCs) {
    npc->removeObserver(this);
    npc->setActive(false);
  }
  activeNPCs.clear();
}
