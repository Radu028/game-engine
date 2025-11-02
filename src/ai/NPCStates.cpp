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
    npc->changeState(std::make_unique<MovingToShopState>());
  }
}

void IdleState::exit(NPC* npc) {}

void MovingToShopState::enter(NPC* npc) {
  auto shop = npc->getTargetShop();
  if (!shop) {
    return;
  }

  npc->sayMessage("greeting");

  Vector3 currentPos = npc->getPosition();

  npc->setDestination(shop->getRandomInteriorPosition());
  hasTarget = true;
  targetPosition = shop->getRandomInteriorPosition();

  hasIntermediateTarget = false;
  currentStage = WaypointStage::SIDE_APPROACH;

  lastPosition = currentPos;
  stuckTimer = 0.0f;
  hasUsedAlternateApproach = false;
}

void MovingToShopState::update(NPC* npc, float deltaTime) {
  auto shop = npc->getTargetShop();
  if (!shop || !hasTarget) {
    npc->changeState(std::make_unique<WanderingState>());
    return;
  }

  Vector3 currentPos = npc->getPosition();
  Vector3 targetPos = npc->getCurrentDestination();

  float distanceToTarget = Vector3Distance(currentPos, targetPos);

  if (distanceToTarget < 5.0f) {
    npc->changeState(std::make_unique<ShoppingState>());
    return;
  }

  npc->moveTowards(targetPos, deltaTime);

  float moveDistance = Vector3Distance(currentPos, lastPosition);
  if (moveDistance < 0.02f) {
    stuckTimer += deltaTime;

    if (stuckTimer > 5.0f) {
      npc->changeState(std::make_unique<WanderingState>());
      return;
    }
  } else {
    stuckTimer = 0.0f;
  }

  lastPosition = currentPos;
}

void MovingToShopState::exit(NPC* npc) {}

void ShoppingState::enter(NPC* npc) {
  shoppingTime = 0.0f;
  fruitSearchTimer = 0.0f;
  shelfLookingTimer = 0.0f;
  chatTimer = 0.0f;
  hasCurrentTarget = false;
  isLookingAtShelf = false;
  hasPurchasedSomething = false;
  shelvesVisited = 0;
  currentShelf = nullptr;
  visitedShelves.clear();

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> timeDist(20.0f, 35.0f);
  maxShoppingTime = timeDist(gen);

  std::uniform_int_distribution<int> shelfDist(2, 4);
  minShelvesToVisit = shelfDist(gen);

  auto shop = npc->getTargetShop();
  if (shop) {
    Vector3 interiorPosition = shop->getRandomInteriorPosition();
    npc->setDestination(interiorPosition);
    currentTarget = interiorPosition;
    hasCurrentTarget = true;
  }

  npc->sayMessage("shopping");
}

void ShoppingState::update(NPC* npc, float deltaTime) {
  shoppingTime += deltaTime;
  fruitSearchTimer += deltaTime;
  chatTimer += deltaTime;

  auto shop = npc->getTargetShop();
  if (!shop) {
    npc->changeState(std::make_unique<WanderingState>());
    return;
  }

  if (chatTimer >= 4.0f) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> chatChance(0.0f, 1.0f);

    if (chatChance(gen) < 0.7f) {
      if (isLookingAtShelf) {
        npc->sayMessage("fruit");
      } else {
        npc->sayMessage("shopping");
      }
    }
    chatTimer = 0.0f;
  }

  if (isLookingAtShelf) {
    shelfLookingTimer += deltaTime;

    if (shelfLookingTimer >= 3.0f) {
      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_real_distribution<float> actionChance(0.0f, 1.0f);

      bool shouldPurchase = false;

      if (!hasPurchasedSomething && shelvesVisited >= minShelvesToVisit) {
        shouldPurchase = actionChance(gen) < 0.8f;
      } else if (hasPurchasedSomething) {
        shouldPurchase = actionChance(gen) < 0.3f;
      } else {
        shouldPurchase = actionChance(gen) < 0.2f;
      }

      if (shouldPurchase && currentShelf) {
        auto availableFruits = currentShelf->getFruits();
        if (!availableFruits.empty()) {
          for (auto fruit : availableFruits) {
            if (fruit && !fruit->getIsPicked()) {
              fruit->pickFruit();
              hasPurchasedSomething = true;

              npc->sayMessage("fruit");
              break;
            }
          }
        }
      }

      isLookingAtShelf = false;
      shelfLookingTimer = 0.0f;
      shelvesVisited++;
      currentShelf = nullptr;
      hasCurrentTarget = false;
    }
    return;
  }

  if (hasCurrentTarget) {
    if (auto navMesh = NPC::getNavMesh()) {
      npc->followPath(deltaTime);
    } else {
      npc->moveTowards(currentTarget, deltaTime);
    }

    Vector3 currentPos = npc->getPosition();
    float distanceToTarget = Vector3Distance(currentPos, currentTarget);

    if (distanceToTarget < 1.5f) {
      hasCurrentTarget = false;
      // Give NPC time to settle at the target before looking for a new one
      fruitSearchTimer = -2.0f;  // Prevents immediate retargeting
    }
  }

  // Look for shelves to examine every 2 seconds or when we don't have a target
  if (fruitSearchTimer >= 2.0f || !hasCurrentTarget) {
    if (!isLookingAtShelf) {
      // Find an UNVISITED shelf to examine
      auto shelves = shop->getShelves();
      std::shared_ptr<class Shelf> targetShelf = nullptr;
      float bestDistance = 1000.0f;

      Vector3 npcPos = npc->getPosition();

      // First priority: find unvisited shelves
      for (auto shelf : shelves) {
        bool alreadyVisited = false;
        for (auto visitedShelf : visitedShelves) {
          if (visitedShelf == shelf) {
            alreadyVisited = true;
            break;
          }
        }

        if (!alreadyVisited) {
          float distance = Vector3Distance(npcPos, shelf->getPosition());
          if (distance <= 8.0f &&
              distance < bestDistance) {  // Increased search range
            targetShelf = shelf;
            bestDistance = distance;
          }
        }
      }

      // If no unvisited shelves in range, allow revisiting if we haven't met
      // minimum
      if (!targetShelf && shelvesVisited < minShelvesToVisit) {
        for (auto shelf : shelves) {
          float distance = Vector3Distance(npcPos, shelf->getPosition());
          if (distance <= 6.0f && distance < bestDistance) {
            targetShelf = shelf;
            bestDistance = distance;
          }
        }
      }

      if (targetShelf) {
        // Move close to shelf and start examining
        Vector3 shelfPos = targetShelf->getPosition();
        Vector3 examinePos = {shelfPos.x, npcPos.y, shelfPos.z + 2.0f};

        // Only set destination if it's significantly different from current
        // target
        if (!hasCurrentTarget ||
            Vector3Distance(examinePos, currentTarget) > 2.0f) {
          currentTarget = examinePos;
          hasCurrentTarget = true;
          npc->setDestination(currentTarget);
        }

        // If very close to shelf, start looking
        if (bestDistance < 2.5f) {
          isLookingAtShelf = true;
          shelfLookingTimer = 0.0f;
          currentShelf = targetShelf;
          hasCurrentTarget = false;

          // Mark this shelf as visited
          visitedShelves.push_back(targetShelf);
        }
      } else if (!hasCurrentTarget) {
        // No shelf nearby, move to a new random location in shop
        Vector3 newInteriorPosition = shop->getRandomInteriorPosition();

        // Only set destination if it's significantly different from last
        // position
        if (Vector3Distance(newInteriorPosition, npcPos) > 2.0f) {
          currentTarget = newInteriorPosition;
          hasCurrentTarget = true;
          npc->setDestination(currentTarget);
        }
      }
    }
    fruitSearchTimer = 0.0f;
  }

  // Finish shopping after the allotted time OR if purchased something and
  // visited enough shelves
  bool shouldLeave = shoppingTime >= maxShoppingTime;
  if (hasPurchasedSomething && shelvesVisited >= minShelvesToVisit) {
    // Can leave early if satisfied with shopping
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> leaveChance(0.0f, 1.0f);
    if (leaveChance(gen) <
        0.1f) {  // 10% chance per second to leave after purchasing
      shouldLeave = true;
    }
  }

  if (shouldLeave) {
    if (!hasPurchasedSomething) {
    } else {
    }
    npc->changeState(std::make_unique<LeavingState>());
  }
}

void ShoppingState::exit(NPC* npc) { hasCurrentTarget = false; }

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
    npc->changeState(std::make_unique<MovingToShopState>());
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
    npc->changeState(std::make_unique<MovingToShopState>());
  }
}

void WanderingState::exit(NPC* npc) { hasWanderTarget = false; }

// LeavingState implementation
void LeavingState::enter(NPC* npc) {
  hasExitTarget = false;
  stuckTimer = 0.0f;
  alternativeAttempts = 0;
  lastPosition = npc->getPosition();

  // First, try to exit the shop by going to a random interior position
  Vector3 currentPos = npc->getPosition();
  if (npc->getTargetShop() && npc->getTargetShop()->isInsideShop(currentPos)) {
    exitTarget = npc->getTargetShop()->getRandomInteriorPosition();
  } else {
    exitTarget = {0.0f, currentPos.y, 0.0f};
  }

  hasExitTarget = true;
  npc->setDestination(exitTarget);
  npc->sayMessage("leaving");
}

void LeavingState::update(NPC* npc, float deltaTime) {
  if (!hasExitTarget) return;

  Vector3 currentPos = npc->getPosition();
  float distanceToExit = Vector3Distance(currentPos, exitTarget);

  float movementDistance = Vector3Distance(currentPos, lastPosition);
  if (movementDistance < 0.02f) {  // Made less sensitive
    stuckTimer += deltaTime;
  } else {
    stuckTimer = 0.0f;
  }
  lastPosition = currentPos;

  if (auto navMesh = NPC::getNavMesh()) {
    npc->followPath(deltaTime);
  } else {
    npc->moveTowards(exitTarget, deltaTime);
  }

  if (stuckTimer > 7.0f) {  // Increased patience from 5.0f to 7.0f
    alternativeAttempts++;

    if (alternativeAttempts <= 3) {
      // Try multiple different exit strategies
      Vector3 shopCenter = {0.0f, currentPos.y, -10.0f};  // Shop is at z=-10
      Vector3 newTarget;

      switch (alternativeAttempts) {
        case 1: {
          // Try moving perpendicular to current direction
          Vector3 awayFromShop = Vector3Subtract(currentPos, shopCenter);
          awayFromShop = Vector3Normalize(awayFromShop);
          Vector3 perpendicular = {-awayFromShop.z, awayFromShop.y,
                                   awayFromShop.x};
          newTarget =
              Vector3Add(currentPos, Vector3Scale(perpendicular, 15.0f));
          break;
        }
        case 2: {
          // Try going to the opposite direction from current target
          Vector3 opposite = Vector3Subtract(currentPos, exitTarget);
          opposite = Vector3Normalize(opposite);
          newTarget = Vector3Add(currentPos, Vector3Scale(opposite, 20.0f));
          break;
        }
        case 3: {
          // Try a completely different direction - go to world corners
          float angle = alternativeAttempts * 90.0f * DEG2RAD;
          newTarget = {cosf(angle) * 25.0f, currentPos.y, sinf(angle) * 25.0f};
          break;
        }
      }

      exitTarget = newTarget;
      exitTarget.y = currentPos.y;  // Keep same height

      npc->setDestination(exitTarget);
      stuckTimer = 0.0f;
    } else {
      // If all alternatives failed, force NPC to deactivate
      npc->setActive(false);
      return;
    }
  }

  if (distanceToExit < 2.0f) {
    if (npc->getTargetShop() &&
        npc->getTargetShop()->isInsideShop(currentPos)) {
    }

    Vector3 shopCenter = {0.0f, currentPos.y, -10.0f};
    float distanceFromShop = Vector3Distance(currentPos, shopCenter);

    if (distanceFromShop > 15.0f) {
      npc->setActive(false);
    } else {
      Vector3 awayFromShop = Vector3Subtract(currentPos, shopCenter);
      awayFromShop = Vector3Normalize(awayFromShop);
      exitTarget = Vector3Add(currentPos, Vector3Scale(awayFromShop, 25.0f));
      exitTarget.y = currentPos.y;

      npc->setDestination(exitTarget);
    }
  }
}

void LeavingState::exit(NPC* npc) { hasExitTarget = false; }