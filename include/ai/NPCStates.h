#pragma once

#include <memory>
#include <vector>

#include "raylib.h"

// Forward declarations
class NPC;
class Shelf;

// State pattern for NPC behavior
class NPCState {
 public:
  virtual ~NPCState() = default;
  virtual void enter(NPC* npc) = 0;
  virtual void update(NPC* npc, float deltaTime) = 0;
  virtual void exit(NPC* npc) = 0;
  virtual std::string getName() = 0;
};

// Concrete states
class IdleState : public NPCState {
 public:
  void enter(NPC* npc) override;
  void update(NPC* npc, float deltaTime) override;
  void exit(NPC* npc) override;
  std::string getName() override { return "Idle"; }

 private:
  float idleTime = 0.0f;
  const float maxIdleTime = 2.0f;  // Increased from 0.5f to 2.0f
};

class MovingToShopState : public NPCState {
 public:
  void enter(NPC* npc) override;
  void update(NPC* npc, float deltaTime) override;
  void exit(NPC* npc) override;
  std::string getName() override { return "MovingToShop"; }

 private:
  enum class WaypointStage {
    SIDE_APPROACH,
    FINAL_APPROACH,
  };

  Vector3 targetPosition;
  bool hasTarget = false;

  // Pathfinding waypoints
  Vector3 intermediateTarget;
  bool hasIntermediateTarget = false;
  WaypointStage currentStage = WaypointStage::SIDE_APPROACH;

  // Stuck detection
  Vector3 lastPosition = {0, 0, 0};
  float stuckTimer = 0.0f;
  bool hasUsedAlternateApproach = false;
};

class ShoppingState : public NPCState {
 public:
  void enter(NPC* npc) override;
  void update(NPC* npc, float deltaTime) override;
  void exit(NPC* npc) override;
  std::string getName() override { return "Shopping"; }

 private:
  float shoppingTime = 0.0f;
  float maxShoppingTime = 8.0f;
  float fruitSearchTimer = 0.0f;
  float shelfLookingTimer = 0.0f;
  float chatTimer = 0.0f;
  Vector3 currentTarget = {0, 0, 0};
  bool hasCurrentTarget = false;
  bool isLookingAtShelf = false;
  bool hasPurchasedSomething = false;
  int shelvesVisited = 0;
  int minShelvesToVisit = 2;
  std::shared_ptr<Shelf> currentShelf = nullptr;
  std::vector<std::shared_ptr<Shelf>> visitedShelves;
};

class WanderingState : public NPCState {
 public:
  void enter(NPC* npc) override;
  void update(NPC* npc, float deltaTime) override;
  void exit(NPC* npc) override;
  std::string getName() override { return "Wandering"; }

 private:
  Vector3 wanderTarget;
  float wanderTime = 0.0f;
  float maxWanderTime = 3.0f;
  bool hasWanderTarget = false;
};

class LeavingState : public NPCState {
 public:
  void enter(NPC* npc) override;
  void update(NPC* npc, float deltaTime) override;
  void exit(NPC* npc) override;
  std::string getName() override { return "Leaving"; }

 private:
  Vector3 exitTarget;
  bool hasExitTarget = false;

  // Stuck detection
  Vector3 lastPosition = {0, 0, 0};
  float stuckTimer = 0.0f;
  int alternativeAttempts = 0;
};
