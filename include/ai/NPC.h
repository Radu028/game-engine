#pragma once

#include <memory>
#include <vector>

#include "ai/NPCChatSystem.h"
#include "ai/NPCStates.h"
#include "ai/NavMesh.h"
#include "entities/HumanoidCharacter.h"
#include "raylib.h"
#include "shop/Fruit.h"
#include "shop/Shop.h"

class GameWorld;
class btDiscreteDynamicsWorld;

// Observer pattern for NPC events
class NPCObserver {
 public:
  virtual ~NPCObserver() = default;
  virtual void onNPCFruitPicked(class NPC* npc,
                                std::shared_ptr<Fruit> fruit) = 0;
  virtual void onNPCExited(class NPC* npc) = 0;
  virtual void onNPCEnteredShop(class NPC* npc) = 0;
};

// NPC controller that drives a humanoid character using composition
class NPC {
 private:
  std::shared_ptr<HumanoidCharacter> character_;
  std::unique_ptr<NPCState> currentState;
  std::shared_ptr<Shop> targetShop;
  std::vector<NPCObserver*> observers;
  NPCChatSystem* chatSystem;

  // AI parameters
  float movementSpeed;
  float detectionRadius;
  float interactionRadius;
  bool isActive;
  float lifetimeTimer;
  float maxLifetime;

  // Pathfinding
  Vector3 currentDestination;
  bool hasDestination;
  std::vector<Vector3> pathWaypoints;
  int currentWaypointIndex;
  static std::shared_ptr<NavMesh> navMesh;  // Shared navigation mesh

  int waypointAttempts;
  int lastWaypointIndex;
  Vector3 lastPosition;
  float stuckTimer;
  bool hasTriedAlternative;

 public:
  NPC(Vector3 position, std::shared_ptr<Shop> shop, GameWorld* world = nullptr);
  ~NPC();

  std::shared_ptr<HumanoidCharacter> getBody() const { return character_; }

  Vector3 getPosition() const;
  Vector3 getTorsoPosition() const;
  Vector3 getFeetPosition() const;
  BoundingBox getBoundingBox() const;

  // State management
  void changeState(std::unique_ptr<NPCState> newState);
  NPCState* getCurrentState() const { return currentState.get(); }
  std::string getCurrentStateName() const {
    return currentState ? currentState->getName() : "None";
  }

  // AI behavior
  void update(float deltaTime);
  void moveTowards(Vector3 target, float deltaTime);
  bool isNearTarget(Vector3 target, float threshold = 1.0f) const;

  // Shop interaction
  std::shared_ptr<Shop> getTargetShop() const { return targetShop; }

  // Pathfinding
  void setDestination(Vector3 destination);
  Vector3 getCurrentDestination() const { return currentDestination; }
  bool hasValidDestination() const { return hasDestination; }
  void followPath(float deltaTime);

  // Navigation mesh
  static void setNavMesh(std::shared_ptr<NavMesh> mesh) { navMesh = mesh; }
  static std::shared_ptr<NavMesh> getNavMesh() { return navMesh; }

  // Observer pattern
  void addObserver(NPCObserver* observer);
  void removeObserver(NPCObserver* observer);
  void notifyFruitPicked(std::shared_ptr<Fruit> fruit);
  void notifyExited();
  void notifyEnteredShop();

  // Utility methods
  std::shared_ptr<Fruit> findNearestFruit() const;

  bool getIsActive() const { return isActive; }

  void setActive(bool active) { isActive = active; }
  void setChatSystem(NPCChatSystem* chat) { chatSystem = chat; }

  void removeFromPhysics(btDiscreteDynamicsWorld* world);

  bool wouldCollideAfterMovement(Vector3 direction, float distance) const;
  float getVerticalCollisionContactTime(const Vector3& verticalMovement,
                                        const GameWorld* world,
                                        int maxIterations) const;

  // Chat system helpers
  void sayMessage(const std::string& context) const;

 private:
  void initializeNPC();
  void updateLifetime(float deltaTime);
  Vector3 calculateRandomColor() const;
  Vector3 findNearestWalkablePosition(Vector3 center, float radius) const;
};
