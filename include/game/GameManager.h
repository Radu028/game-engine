#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "ai/NPCChatSystem.h"
#include "ai/NPCManager.h"
#include "raylib.h"
#include "shop/Shop.h"

// Enum for different game states
// Game management class using Singleton pattern
class GameManager {
 private:
  static GameManager* instance;

  Camera3D camera;
  std::shared_ptr<Shop> shop;
  NPCManager* npcManager;
  std::unique_ptr<NPCChatSystem> chatSystem;

  // Game parameters
  int initialStockPerFruit;
  int totalFruitsInStock;
  float gameTimer;

  // Statistics
  int fruitsPickedByNPCs;
  int npcsSoFar;

  std::unordered_map<std::string, Texture2D> npcStatusTextures;
  float npcStatusBillboardWidth;
  float npcStatusVerticalOffset;
  float npcStatusFontSize;
  int npcStatusPadding;

  GameManager();

 public:
  ~GameManager();

  // Singleton access
  static GameManager* getInstance();
  static void destroyInstance();

  // Game lifecycle
  void update(float deltaTime);
  void render(Camera3D camera);

  // Game state management
  // Shop access
  std::shared_ptr<Shop> getShop() const { return shop; }
  void setShop(std::shared_ptr<Shop> newShop);

  // Game status
  int getRemainingFruits() const;

  // Statistics
  int getFruitsPickedByNPCs() const { return fruitsPickedByNPCs; }
  int getNPCCount() const;
  float getGameTime() const { return gameTimer; }

  // Game events
  void onFruitPicked();
  void onNPCSpawned();

  // Chat system access
  NPCChatSystem* getChatSystem() const { return chatSystem.get(); }

 private:
  void updateGameLogic(float deltaTime);
  void initializeShop();
  void drawNPCStatusBillboard(const std::shared_ptr<NPC>& npc,
                              const Camera3D& camera);
  Texture2D& getOrCreateStatusTexture(const std::string& stateText,
                                      float fontSize, int padding);
};
