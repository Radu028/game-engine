#include "ai/NPCStates.h"

#include <cmath>
#include <random>

#include "ai/NPC.h"
#include "raymath.h"
#include "settings/Physics.h"
#include "shop/Shelf.h"

namespace {
std::mt19937& rng() {
  static std::mt19937 gen(std::random_device{}());
  return gen;
}

float rand01() {
  static std::uniform_real_distribution<float> dist(0.0f, 1.0f);
  return dist(rng());
}

float randRange(float min, float max) {
  std::uniform_real_distribution<float> dist(min, max);
  return dist(rng());
}

size_t randIndex(size_t n) {
  std::uniform_int_distribution<size_t> dist(0, n - 1);
  return dist(rng());
}
}  // namespace

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
    handleShelfLooking(npc, deltaTime,
                       GameSettings::NPC::LOOKING_AT_SHELF_TIME);
    return;
  }

  // Ensure we always have a shelf target while shopping.
  if (!currentShelf) {
    handleShelfSearch(npc);
  }

  // When we have a shelf target, detect arrival and transition to "looking".
  handleMovement(npc, deltaTime);

  if (shouldLeaveShop(npc)) {
    npc->changeState(std::make_unique<LeavingState>());
  }
}

void ShoppingState::exit(NPC* npc) { currentShelf = nullptr; }

void ShoppingState::handleChatting(NPC* npc) {
  if (chatTimer < GameSettings::NPC::CHAT_INTERVAL) return;

  if (rand01() < 0.7f) {
    npc->sayMessage(isLookingAtShelf ? "fruit" : "shopping");
  }
  chatTimer = 0.0f;
}

void ShoppingState::handleShelfLooking(NPC* npc, float deltaTime,
                                       float lookingAtShelfTime) {
  shelfLookingTimer += deltaTime;

  if (currentShelf && currentShelf->isEmpty()) {
    isLookingAtShelf = false;
    shelfLookingTimer = 0.0f;
    currentShelf = nullptr;
    return;
  }

  if (shelfLookingTimer < lookingAtShelfTime) return;

  if (shouldPurchaseFruit()) {
    attemptFruitPurchase(npc);
  }

  isLookingAtShelf = false;
  shelfLookingTimer = 0.0f;
  shelvesVisited++;
  currentShelf = nullptr;
}

bool ShoppingState::shouldPurchaseFruit() const {
  if (!hasPurchasedSomething && shelvesVisited >= minShelvesToVisit) {
    return rand01() < 0.8f;
  }
  if (hasPurchasedSomething) {
    return rand01() < 0.3f;
  }
  return rand01() < 0.2f;
}

void ShoppingState::attemptFruitPurchase(NPC* npc) {
  if (!currentShelf) return;

  auto availableFruits = currentShelf->getFruits();
  if (availableFruits.empty()) return;

  for (auto fruit : availableFruits) {
    if (fruit && !fruit->getIsPicked()) {
      fruit->pickFruit();
      npc->notifyFruitPicked(fruit);
      hasPurchasedSomething = true;
      npc->sayMessage("fruit");
      break;
    }
  }
}

void ShoppingState::handleMovement(NPC* npc, float deltaTime) {
  if (!currentShelf) return;

  // If the shelf became empty (someone else bought everything), pick another.
  if (currentShelf->isEmpty()) {
    currentShelf = nullptr;
    return;
  }

  Vector3 currentPos = npc->getPosition();
  float distanceToTarget = Vector3Distance(currentPos, currentTarget);

  // Once we're close enough, switch into the "looking at shelf" phase.
  if (distanceToTarget < GameSettings::NPC::ARRIVAL_THRESHOLD) {
    isLookingAtShelf = true;
    shelfLookingTimer = 0.0f;
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
  (void)npc;
  if (shelves.empty()) return nullptr;

  std::vector<std::shared_ptr<Shelf>> candidates;
  candidates.reserve(shelves.size());
  for (const auto& shelf : shelves) {
    if (!shelf) continue;
    if (shelf == currentShelf) continue;
    if (shelf->isEmpty()) continue;
    candidates.push_back(shelf);
  }

  const auto& pool = !candidates.empty() ? candidates : shelves;
  // If we fell back to `shelves`, make sure we pick a non-null shelf.
  for (int tries = 0; tries < 8; ++tries) {
    auto picked = pool[randIndex(pool.size())];
    if (picked) return picked;
  }
  for (const auto& shelf : shelves) {
    if (shelf) return shelf;
  }
  return nullptr;
}

void ShoppingState::moveToShelf(NPC* npc, std::shared_ptr<Shelf> shelf) {
  Vector3 npcPos = npc->getPosition();
  Vector3 shelfPos = shelf->getPosition();

  currentTarget = shelfPos;
  currentShelf = shelf;
  npc->setDestination(currentTarget);

  // If very close to shelf, start looking
  if (Vector3Distance(npcPos, shelfPos) <
      GameSettings::NPC::ARRIVAL_THRESHOLD) {
    isLookingAtShelf = true;
    shelfLookingTimer = 0.0f;
  }
}

bool ShoppingState::shouldLeaveShop(NPC* npc) const {
  if (shoppingTime >= maxShoppingTime) return true;

  if (!hasPurchasedSomething || shelvesVisited < minShelvesToVisit) {
    return false;
  }

  return rand01() < GameSettings::NPC::LEAVE_PROBABILITY;
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
    Vector3 toTarget = Vector3Subtract(targetPos, currentPos);
    toTarget = Vector3Normalize(toTarget);

    float angle = randRange(-PI / 3, PI / 3);

    Vector3 wanderDirection = {
        toTarget.x * cos(angle) - toTarget.z * sin(angle), 0,
        toTarget.x * sin(angle) + toTarget.z * cos(angle)};
    wanderDirection = Vector3Normalize(wanderDirection);

    float wanderDistance = randRange(5.0f, 12.0f);

    wanderTarget =
        Vector3Add(currentPos, Vector3Scale(wanderDirection, wanderDistance));
    wanderTarget.y = currentPos.y;

    hasWanderTarget = true;
    wanderTime = 0.0f;
    npc->setDestination(wanderTarget);
  }

  if (wanderTime > GameSettings::NPC::MAX_WANDER_TIME + 3.0f) {
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

  // Calculate exit position outside the shop, not inside
  // Move away from shop center towards exterior
  Vector3 shopCenter = {0.0f, currentPos.y, -10.0f};
  Vector3 awayFromShop = Vector3Subtract(currentPos, shopCenter);
  awayFromShop = Vector3Normalize(awayFromShop);
  exitTarget = Vector3Add(currentPos, Vector3Scale(awayFromShop, 20.0f));
  exitTarget.y = currentPos.y;

  hasExitTarget = true;
  npc->setDestination(exitTarget);
  npc->sayMessage("leaving");
}

void LeavingState::update(NPC* npc, float deltaTime) {
  if (!hasExitTarget) return;

  Vector3 currentPos = npc->getPosition();
  float distanceToExit = Vector3Distance(currentPos, exitTarget);

  updateStuckDetection(npc, deltaTime);

  if (stuckTimer > GameSettings::NPC::STUCK_TIMEOUT) {
    handleStuckSituation(npc);
    return;
  }

  if (distanceToExit < GameSettings::NPC::EXIT_DISTANCE) {
    handleReachedExit(npc, currentPos);
  }
}

void LeavingState::exit(NPC* npc) { hasExitTarget = false; }

void LeavingState::updateStuckDetection(NPC* npc, float deltaTime) {
  Vector3 currentPos = npc->getPosition();
  float movementDistance = Vector3Distance(currentPos, lastPosition);

  if (movementDistance < GameSettings::NPC::STUCK_THRESHOLD) {
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
    npc->notifyExited();
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

  if (distanceFromShop > GameSettings::NPC::SHOP_EXIT_DISTANCE) {
    npc->setActive(false);
    npc->notifyExited();
    return;
  }

  Vector3 awayFromShop = Vector3Subtract(currentPos, shopCenter);
  awayFromShop = Vector3Normalize(awayFromShop);
  exitTarget = Vector3Add(currentPos, Vector3Scale(awayFromShop, 25.0f));
  exitTarget.y = currentPos.y;

  npc->setDestination(exitTarget);
}