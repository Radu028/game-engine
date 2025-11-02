#include "ai/NPCStates.h"

#include <cmath>
#include <random>

#include "ai/NPC.h"
#include "raymath.h"
#include "shop/Shelf.h"

void IdleState::enter(NPC* npc) {
  idleTime = 0.0f;

  npc->sayMessage("greeting");
}

void IdleState::update(NPC* npc, float deltaTime) {
  idleTime += deltaTime;

  if (idleTime >= maxIdleTime) {
    npc->changeState(std::make_unique<ShoppingState>());
  }
}

void IdleState::exit(NPC* npc) {}

void ShoppingState::enter(NPC* npc) {
  shoppingTime = 0.0f;
  fruitSearchTimer = 0.0f;
  shelfLookingTimer = 0.0f;
  chatTimer = 0.0f;
  isLookingAtShelf = false;
  hasPurchasedSomething = false;
  shelvesVisited = 0;
  currentShelf = nullptr;
}

void ShoppingState::update(NPC* npc, float deltaTime) {
  shoppingTime += deltaTime;
  fruitSearchTimer += deltaTime;
  chatTimer += deltaTime;

  auto shelves = npc->getTargetShop()->getShelves();
  if (shelves.empty()) {
    npc->changeState(std::make_unique<WanderingState>());
    return;
  }

  handleChatting(npc);

  if (isLookingAtShelf) {
    handleShelfLooking(npc, deltaTime);
    return;
  }

  handleMovement(npc, deltaTime);

  if (fruitSearchTimer >= 2.0f || !currentShelf) {
    handleShelfSearch(npc);
  }

  if (shouldLeaveShop(npc)) {
    npc->changeState(std::make_unique<LeavingState>());
  }
}

void ShoppingState::exit(NPC* npc) { currentShelf = nullptr; }

void ShoppingState::handleChatting(NPC* npc) {
  if (chatTimer < 4.0f) return;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> chatChance(0.0f, 1.0f);

  if (chatChance(gen) < 0.7f) {
    npc->sayMessage(isLookingAtShelf ? "fruit" : "shopping");
  }
  chatTimer = 0.0f;
}

void ShoppingState::handleShelfLooking(NPC* npc, float deltaTime) {
  shelfLookingTimer += deltaTime;

  if (shelfLookingTimer < 3.0f) return;

  if (shouldPurchaseFruit()) {
    attemptFruitPurchase(npc);
  }

  isLookingAtShelf = false;
  shelfLookingTimer = 0.0f;
  shelvesVisited++;
  currentShelf = nullptr;
}

bool ShoppingState::shouldPurchaseFruit() const {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> actionChance(0.0f, 1.0f);

  if (!hasPurchasedSomething && shelvesVisited >= minShelvesToVisit) {
    return actionChance(gen) < 0.8f;
  }
  if (hasPurchasedSomething) {
    return actionChance(gen) < 0.3f;
  }
  return actionChance(gen) < 0.2f;
}

void ShoppingState::attemptFruitPurchase(NPC* npc) {
  if (!currentShelf) return;

  auto availableFruits = currentShelf->getFruits();
  if (availableFruits.empty()) return;

  for (auto fruit : availableFruits) {
    if (fruit && !fruit->getIsPicked()) {
      fruit->pickFruit();
      hasPurchasedSomething = true;
      npc->sayMessage("fruit");
      break;
    }
  }
}

void ShoppingState::handleMovement(NPC* npc, float deltaTime) {
  if (!currentShelf) return;

  if (auto navMesh = NPC::getNavMesh()) {
    npc->followPath(deltaTime);
  } else {
    npc->moveTowards(currentTarget, deltaTime);
  }

  Vector3 currentPos = npc->getPosition();
  float distanceToTarget = Vector3Distance(currentPos, currentTarget);

  if (distanceToTarget < 1.0f) {
    currentShelf = nullptr;
    fruitSearchTimer = -2.0f;  // Prevents immediate retargeting
  }
}

void ShoppingState::handleShelfSearch(NPC* npc) {
  if (isLookingAtShelf) return;

  auto shelves = npc->getTargetShop()->getShelves();
  auto targetShelf = findTargetShelf(npc, shelves);

  if (targetShelf) {
    moveToShelf(npc, targetShelf);
  } else {
    npc->changeState(std::make_unique<IdleState>());
  }

  fruitSearchTimer = 0.0f;
}

std::shared_ptr<Shelf> ShoppingState::findTargetShelf(
    NPC* npc, const std::vector<std::shared_ptr<Shelf>>& shelves) {
  return shelves[rand() % shelves.size()];
}

void ShoppingState::moveToShelf(NPC* npc, std::shared_ptr<Shelf> shelf) {
  Vector3 npcPos = npc->getPosition();
  Vector3 shelfPos = shelf->getPosition();
  Vector3 examinePos = {shelfPos.x, npcPos.y, shelfPos.z + 2.0f};

  // Only set destination if it's significantly different
  if (Vector3Distance(examinePos, currentTarget) > 2.0f) {
    currentTarget = examinePos;
    npc->setDestination(currentTarget);
  }

  // If very close to shelf, start looking
  if (Vector3Distance(examinePos, npcPos) < 2.5f) {
    isLookingAtShelf = true;
    shelfLookingTimer = 0.0f;
    currentShelf = shelf;
  }
}

bool ShoppingState::shouldLeaveShop(NPC* npc) const {
  if (shoppingTime >= maxShoppingTime) return true;

  if (!hasPurchasedSomething || shelvesVisited < minShelvesToVisit) {
    return false;
  }

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> leaveChance(0.0f, 1.0f);
  return leaveChance(gen) < 0.1f;
}

// WanderingState implementation
void WanderingState::enter(NPC* npc) {
  wanderTime = 0.0f;
  hasWanderTarget = false;
}

void WanderingState::update(NPC* npc, float deltaTime) {
  wanderTime += deltaTime;

  auto shop = npc->getTargetShop();
  if (!shop) {
    npc->setActive(false);
    return;
  }

  Vector3 currentPos = npc->getPosition();
  Vector3 targetPos = shop->getRandomInteriorPosition();

  float distanceToTarget = Vector3Distance(currentPos, targetPos);
  if (distanceToTarget < 8.0f) {
    npc->changeState(std::make_unique<ShoppingState>());
    return;
  }

  if (!hasWanderTarget || wanderTime >= maxWanderTime) {
    std::random_device rd;
    std::mt19937 gen(rd());

    Vector3 toTarget = Vector3Subtract(targetPos, currentPos);
    toTarget = Vector3Normalize(toTarget);

    std::uniform_real_distribution<float> angleDist(-PI / 3, PI / 3);
    float angle = angleDist(gen);

    Vector3 wanderDirection = {
        toTarget.x * cos(angle) - toTarget.z * sin(angle), 0,
        toTarget.x * sin(angle) + toTarget.z * cos(angle)};
    wanderDirection = Vector3Normalize(wanderDirection);

    std::uniform_real_distribution<float> distDist(5.0f, 12.0f);
    float wanderDistance = distDist(gen);

    wanderTarget =
        Vector3Add(currentPos, Vector3Scale(wanderDirection, wanderDistance));
    wanderTarget.y = currentPos.y;

    hasWanderTarget = true;
    wanderTime = 0.0f;
    npc->setDestination(wanderTarget);
  }

  if (wanderTime > 8.0f) {
    npc->changeState(std::make_unique<ShoppingState>());
  }
}

void WanderingState::exit(NPC* npc) { hasWanderTarget = false; }

// LeavingState implementation
void LeavingState::enter(NPC* npc) {
  hasExitTarget = false;
  stuckTimer = 0.0f;
  alternativeAttempts = 0;
  lastPosition = npc->getPosition();

  Vector3 currentPos = npc->getPosition();
  exitTarget = npc->getTargetShop()->getRandomInteriorPosition();

  hasExitTarget = true;
  npc->setDestination(exitTarget);
  npc->sayMessage("leaving");
}

void LeavingState::update(NPC* npc, float deltaTime) {
  if (!hasExitTarget) return;

  Vector3 currentPos = npc->getPosition();
  float distanceToExit = Vector3Distance(currentPos, exitTarget);

  updateStuckDetection(npc, deltaTime);

  if (auto navMesh = NPC::getNavMesh()) {
    npc->followPath(deltaTime);
  } else {
    npc->moveTowards(exitTarget, deltaTime);
  }

  if (stuckTimer > 7.0f) {
    handleStuckSituation(npc);
    return;
  }

  if (distanceToExit < 2.0f) {
    handleReachedExit(npc, currentPos);
  }
}

void LeavingState::exit(NPC* npc) { hasExitTarget = false; }

void LeavingState::updateStuckDetection(NPC* npc, float deltaTime) {
  Vector3 currentPos = npc->getPosition();
  float movementDistance = Vector3Distance(currentPos, lastPosition);

  if (movementDistance < 0.02f) {
    stuckTimer += deltaTime;
  } else {
    stuckTimer = 0.0f;
  }
  lastPosition = currentPos;
}

void LeavingState::handleStuckSituation(NPC* npc) {
  alternativeAttempts++;

  if (alternativeAttempts > 3) {
    npc->setActive(false);
    return;
  }

  Vector3 currentPos = npc->getPosition();
  Vector3 shopCenter = {0.0f, currentPos.y, -10.0f};
  Vector3 newTarget = calculateAlternativeTarget(npc, currentPos, shopCenter);

  exitTarget = newTarget;
  exitTarget.y = currentPos.y;

  npc->setDestination(exitTarget);
  stuckTimer = 0.0f;
}

Vector3 LeavingState::calculateAlternativeTarget(NPC* npc, Vector3 currentPos,
                                                 Vector3 shopCenter) {
  switch (alternativeAttempts) {
    case 1: {
      Vector3 awayFromShop = Vector3Subtract(currentPos, shopCenter);
      awayFromShop = Vector3Normalize(awayFromShop);
      Vector3 perpendicular = {-awayFromShop.z, awayFromShop.y, awayFromShop.x};
      return Vector3Add(currentPos, Vector3Scale(perpendicular, 15.0f));
    }
    case 2: {
      Vector3 opposite = Vector3Subtract(currentPos, exitTarget);
      opposite = Vector3Normalize(opposite);
      return Vector3Add(currentPos, Vector3Scale(opposite, 20.0f));
    }
    case 3: {
      float angle = alternativeAttempts * 90.0f * DEG2RAD;
      return {cosf(angle) * 25.0f, currentPos.y, sinf(angle) * 25.0f};
    }
    default:
      return currentPos;
  }
}

void LeavingState::handleReachedExit(NPC* npc, Vector3 currentPos) {
  Vector3 shopCenter = {0.0f, currentPos.y, -10.0f};
  float distanceFromShop = Vector3Distance(currentPos, shopCenter);

  if (distanceFromShop > 15.0f) {
    npc->setActive(false);
    return;
  }

  Vector3 awayFromShop = Vector3Subtract(currentPos, shopCenter);
  awayFromShop = Vector3Normalize(awayFromShop);
  exitTarget = Vector3Add(currentPos, Vector3Scale(awayFromShop, 25.0f));
  exitTarget.y = currentPos.y;

  npc->setDestination(exitTarget);
}