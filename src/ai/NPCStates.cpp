#include "ai/NPCStates.h"

#include <algorithm>
#include <cmath>
#include <random>

#include "GameWorld.h"
#include "ai/NPC.h"
#include "raymath.h"
#include "settings/Physics.h"
#include "shop/Shelf.h"
#include "shop/Shop.h"

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

Vector3 normalizeOrFallback(Vector3 value, Vector3 fallback = {0.0f, 0.0f, 1.0f}) {
  float len = Vector3Length(value);
  if (len > 0.0001f) {
    return Vector3Scale(value, 1.0f / len);
  }

  float fallbackLen = Vector3Length(fallback);
  if (fallbackLen > 0.0001f) {
    return Vector3Scale(fallback, 1.0f / fallbackLen);
  }

  return {0.0f, 0.0f, 1.0f};
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
    // Go to entrance first, then start shopping
    npc->changeState(std::make_unique<EnteringState>());
  }
}

void IdleState::exit(NPC* /*npc*/) {}

void ShoppingState::enter(NPC* /*npc*/) {
  shoppingTime = 0.0f;
  maxShoppingTime = GameSettings::NPC::MAX_SHOPPING_TIME;
  fruitSearchTimer = 0.0f;
  shelfLookingTimer = 0.0f;
  chatTimer = 0.0f;
  isLookingAtShelf = false;
  hasPurchasedSomething = false;
  shelvesVisited = 0;
  minShelvesToVisit = GameSettings::NPC::MIN_SHELVES_TO_VISIT;
  currentShelf = nullptr;
}

void ShoppingState::update(NPC* npc, float deltaTime) {
  shoppingTime += deltaTime;
  fruitSearchTimer += deltaTime;
  chatTimer += deltaTime;

  auto shop = npc->getTargetShop();
  if (!shop) {
    npc->setActive(false);
    return;
  }

  const auto& shelves = shop->getShelves();
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

void ShoppingState::exit(NPC* /*npc*/) { currentShelf = nullptr; }

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
  // More likely to purchase - simulates real shopping behavior
  if (!hasPurchasedSomething) {
    return rand01() < 0.9f;  // 90% chance to buy first item
  }
  // After first purchase, 50% chance to buy more
  return rand01() < 0.5f;
}

void ShoppingState::attemptFruitPurchase(NPC* npc) {
  if (!currentShelf) return;

  const auto& availableFruits = currentShelf->getFruits();
  if (availableFruits.empty()) return;

  for (auto fruit : availableFruits) {
    if (fruit && !fruit->getIsPicked()) {
      fruit->pickFruit();
      currentShelf->removeFruit(fruit);
      if (GameWorld* world = GameWorld::getInstance()) {
        world->removeObject(fruit);
      }
      npc->notifyFruitPicked(fruit);
      hasPurchasedSomething = true;
      npc->sayMessage("fruit");
      break;
    }
  }
}

void ShoppingState::handleMovement(NPC* npc, float /*deltaTime*/) {
  if (!currentShelf) return;

  if (!npc->hasValidDestination()) {
    if (!npc->setDestination(currentTarget)) {
      currentShelf = nullptr;
    }
    return;
  }

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

  auto shop = npc->getTargetShop();
  if (!shop) {
    npc->setActive(false);
    return;
  }

  const auto& shelves = shop->getShelves();
  auto targetShelf = findTargetShelf(npc, shelves);

  if (targetShelf) {
    moveToShelf(npc, targetShelf);
  } else {
    npc->changeState(std::make_unique<LeavingState>());
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

  if (candidates.empty()) {
    return nullptr;
  }

  for (int tries = 0; tries < 8; ++tries) {
    auto picked = candidates[randIndex(candidates.size())];
    if (picked) {
      return picked;
    }
  }
  for (const auto& shelf : candidates) {
    if (shelf) {
      return shelf;
    }
  }
  return nullptr;
}

void ShoppingState::moveToShelf(NPC* npc, std::shared_ptr<Shelf> shelf) {
  Vector3 npcPos = npc->getPosition();
  Vector3 shelfPos = shelf->getPosition();

  currentTarget = shelfPos;
  currentShelf = shelf;
  if (!npc->setDestination(currentTarget)) {
    currentShelf = nullptr;
    return;
  }

  // If very close to shelf, start looking
  if (Vector3Distance(npcPos, shelfPos) <
      GameSettings::NPC::ARRIVAL_THRESHOLD) {
    isLookingAtShelf = true;
    shelfLookingTimer = 0.0f;
  }
}

bool ShoppingState::shouldLeaveShop(NPC* /*npc*/) const {
  if (shoppingTime >= maxShoppingTime) return true;

  if (!hasPurchasedSomething || shelvesVisited < minShelvesToVisit) {
    return false;
  }

  return rand01() < GameSettings::NPC::LEAVE_PROBABILITY;
}

// WanderingState implementation
void WanderingState::enter(NPC* /*npc*/) {
  wanderTime = 0.0f;
  maxWanderTime = GameSettings::NPC::MAX_WANDER_TIME;
  hasWanderTarget = false;
}

void WanderingState::update(NPC* npc, float deltaTime) {
  wanderTime += deltaTime;

  auto shop = npc->getTargetShop();
  if (!shop) {
    npc->setActive(false);
    return;
  }

  if (wanderTime >= maxWanderTime) {
    npc->changeState(std::make_unique<ShoppingState>());
    return;
  }

  if (!hasWanderTarget) {
    Vector3 currentPos = npc->getPosition();
    Vector3 targetPos = shop->getRandomInteriorPosition();
    Vector3 toTarget = Vector3Subtract(targetPos, currentPos);
    toTarget = normalizeOrFallback(toTarget);

    float angle = randRange(-PI / 3, PI / 3);

    Vector3 wanderDirection = {
        toTarget.x * cosf(angle) - toTarget.z * sinf(angle), 0.0f,
        toTarget.x * sinf(angle) + toTarget.z * cosf(angle)};
    wanderDirection = normalizeOrFallback(wanderDirection);

    float wanderDistance = randRange(5.0f, 12.0f);

    wanderTarget =
        Vector3Add(currentPos, Vector3Scale(wanderDirection, wanderDistance));
    wanderTarget.y = currentPos.y;

    hasWanderTarget = true;
    if (!npc->setDestination(wanderTarget)) {
      hasWanderTarget = false;
    }
  } else if (!npc->hasValidDestination()) {
    if (!npc->setDestination(wanderTarget)) {
      hasWanderTarget = false;
    }
  }

  if (npc->isNearTarget(wanderTarget, GameSettings::NPC::ARRIVAL_THRESHOLD)) {
    npc->changeState(std::make_unique<ShoppingState>());
  }
}

void WanderingState::exit(NPC* /*npc*/) { hasWanderTarget = false; }

// EnteringState implementation
void EnteringState::enter(NPC* npc) {
  enteringTime = 0.0f;
  stuckTimer = 0.0f;
  lastPosition = npc->getPosition();
  retryCount = 0;
  phase = 0;

  auto shop = npc->getTargetShop();
  if (!shop) {
    npc->setActive(false);
    return;
  }

  // First, walk to a position in front of the entrance
  Vector3 exitPos = shop->getExitPosition();
  entranceTarget = {exitPos.x, 0.5f, exitPos.z};
  npc->setDestination(entranceTarget);
  npc->sayMessage("greeting");
}

void EnteringState::update(NPC* npc, float deltaTime) {
  enteringTime += deltaTime;

  auto shop = npc->getTargetShop();
  if (!shop) {
    npc->setActive(false);
    return;
  }

  Vector3 currentPos = npc->getPosition();

  // Check if already inside shop - success!
  if (shop->isInsideShop(currentPos)) {
    npc->notifyEnteredShop();
    npc->changeState(std::make_unique<ShoppingState>());
    return;
  }

  float distanceToTarget = Vector3Distance(currentPos, entranceTarget);

  if (!npc->hasValidDestination()) {
    npc->setDestination(entranceTarget);
  }

  // State machine for entering
  if (phase == 0 && distanceToTarget < 2.0f) {
    // Phase 0 complete: reached front of entrance, now go to entrance
    phase = 1;
    entranceTarget = shop->getEntrancePosition();
    npc->setDestination(entranceTarget);
    stuckTimer = 0.0f;
    return;
  }

  if (phase == 1 && distanceToTarget < 1.5f) {
    // Phase 1 complete: at entrance, now move inside
    phase = 2;
    Vector3 insidePos = shop->getEntrancePosition();
    insidePos.z -= 3.0f;  // Move deeper into the shop
    entranceTarget = insidePos;
    npc->setDestination(entranceTarget);
    stuckTimer = 0.0f;
    return;
  }

  // Stuck detection
  float movementDistance = Vector3Distance(currentPos, lastPosition);
  if (movementDistance < GameSettings::NPC::STUCK_THRESHOLD) {
    stuckTimer += deltaTime;
  } else {
    stuckTimer = std::max(0.0f, stuckTimer - deltaTime * 0.3f);
  }
  lastPosition = currentPos;

  // If stuck, try alternative approaches based on current phase
  if (stuckTimer > GameSettings::NPC::STUCK_TIMEOUT) {
    retryCount++;
    stuckTimer = 0.0f;

    if (retryCount > 8) {
      npc->setActive(false);
      return;
    }

    // Try different approach based on phase
    Vector3 shopPos = shop->getPosition();
    Vector3 entrancePos = shop->getEntrancePosition();

    if (phase == 0) {
      // Try approaching from different angles
      float xOffset = (retryCount % 2 == 0) ? 2.0f : -2.0f;
      xOffset *= (retryCount / 2 + 1);
      entranceTarget = {shopPos.x + xOffset, 0.5f, entrancePos.z + 4.0f};
    } else {
      // Try sliding into the entrance from the side
      float xOffset = (retryCount % 2 == 0) ? 1.0f : -1.0f;
      entranceTarget = {entrancePos.x + xOffset, 0.5f, entrancePos.z};
    }

    npc->setDestination(entranceTarget);
  }

  // Timeout
  if (enteringTime > maxEnteringTime) {
    npc->setActive(false);
  }
}

void EnteringState::exit(NPC* /*npc*/) {}

// LeavingState implementation
void LeavingState::enter(NPC* npc) {
  hasExitTarget = false;
  stuckTimer = 0.0f;
  alternativeAttempts = 0;
  lastPosition = npc->getPosition();

  auto shop = npc->getTargetShop();
  if (!shop) {
    npc->setActive(false);
    return;
  }

  // Use the shop's exit position method
  exitTarget = shop->getExitPosition();

  hasExitTarget = true;
  npc->setDestination(exitTarget);
  npc->sayMessage("leaving");
}

void LeavingState::update(NPC* npc, float deltaTime) {
  if (!hasExitTarget) return;

  Vector3 currentPos = npc->getPosition();
  float distanceToExit = Vector3Distance(currentPos, exitTarget);

  if (!npc->hasValidDestination()) {
    npc->setDestination(exitTarget);
  }

  updateStuckDetection(npc, deltaTime);

  if (stuckTimer > GameSettings::NPC::STUCK_TIMEOUT) {
    handleStuckSituation(npc);
    return;
  }

  if (distanceToExit < GameSettings::NPC::EXIT_DISTANCE) {
    handleReachedExit(npc, currentPos);
  }
}

void LeavingState::exit(NPC* /*npc*/) { hasExitTarget = false; }

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
    return;
  }

  auto shop = npc->getTargetShop();
  Vector3 currentPos = npc->getPosition();
  Vector3 shopCenter = shop ? shop->getPosition() : Vector3{0.0f, 0.0f, -10.0f};
  shopCenter.y = currentPos.y;

  Vector3 newTarget = calculateAlternativeTarget(npc, currentPos, shopCenter);

  exitTarget = newTarget;
  exitTarget.y = currentPos.y;

  npc->setDestination(exitTarget);
  stuckTimer = 0.0f;
}

Vector3 LeavingState::calculateAlternativeTarget(NPC* /*npc*/, Vector3 currentPos,
                                                 Vector3 shopCenter) {
  switch (alternativeAttempts) {
    case 1: {
      Vector3 awayFromShop =
          normalizeOrFallback(Vector3Subtract(currentPos, shopCenter));
      Vector3 perpendicular = {-awayFromShop.z, 0.0f, awayFromShop.x};
      return Vector3Add(currentPos, Vector3Scale(perpendicular, 15.0f));
    }
    case 2: {
      Vector3 opposite = Vector3Subtract(currentPos, exitTarget);
      opposite = normalizeOrFallback(
          opposite, normalizeOrFallback(Vector3Subtract(currentPos, shopCenter)));
      return Vector3Add(currentPos, Vector3Scale(opposite, 20.0f));
    }
    case 3: {
      Vector3 awayFromShop =
          normalizeOrFallback(Vector3Subtract(currentPos, shopCenter));
      return Vector3Add(currentPos, Vector3Scale(awayFromShop, 30.0f));
    }
    default:
      return currentPos;
  }
}

void LeavingState::handleReachedExit(NPC* npc, Vector3 currentPos) {
  auto shop = npc->getTargetShop();
  Vector3 shopCenter = shop ? shop->getPosition() : Vector3{0.0f, 0.0f, -10.0f};
  shopCenter.y = currentPos.y;

  float distanceFromShop = Vector3Distance(currentPos, shopCenter);

  if (distanceFromShop > GameSettings::NPC::SHOP_EXIT_DISTANCE) {
    npc->setActive(false);
    return;
  }

  // Continue moving away from shop
  Vector3 awayFromShop = Vector3Subtract(currentPos, shopCenter);
  awayFromShop = normalizeOrFallback(awayFromShop);
  exitTarget = Vector3Add(currentPos, Vector3Scale(awayFromShop, 25.0f));
  exitTarget.y = currentPos.y;

  npc->setDestination(exitTarget);
}
