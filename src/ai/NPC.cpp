#include "ai/NPC.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>

#include "GameWorld.h"
#include "ai/NPCStates.h"
#include "raymath.h"
#include "settings/Physics.h"

std::shared_ptr<NavMesh> NPC::navMesh = nullptr;

NPC::NPC(Vector3 position, std::shared_ptr<Shop> shop)
    : HumanoidCharacter(position),
      targetShop(shop),
      chatSystem(nullptr),
      movementSpeed(3.0f),
      detectionRadius(5.0f),
      interactionRadius(1.0f),
      isActive(true),
      lifetimeTimer(0.0f),
      maxLifetime(60.0f),
      hasDestination(false),
      currentWaypointIndex(0),
      waypointAttempts(0),
      lastWaypointIndex(-1),
      lastPosition(position),
      stuckTimer(0.0f),
      hasTriedAlternative(false) {
  initializeNPC();

  currentState = std::make_unique<IdleState>();
  currentState->enter(this);
}

NPC::~NPC() {
  if (currentState) {
    currentState->exit(this);
  }
}

void NPC::initializeNPC() {
  static std::random_device rd;
  static std::mt19937 gen(rd());
  std::uniform_real_distribution<float> speedDist(2.5f, 4.0f);

  movementSpeed = speedDist(gen);

  Vector3 torsoColorVec = calculateRandomColor();
  Vector3 armColorVec = calculateRandomColor();
  Vector3 legColorVec = calculateRandomColor();

  Color torsoColor = {static_cast<unsigned char>(torsoColorVec.x * 255),
                      static_cast<unsigned char>(torsoColorVec.y * 255),
                      static_cast<unsigned char>(torsoColorVec.z * 255), 255};

  Color armColor = {static_cast<unsigned char>(armColorVec.x * 255),
                    static_cast<unsigned char>(armColorVec.y * 255),
                    static_cast<unsigned char>(armColorVec.z * 255), 255};

  Color legColor = {static_cast<unsigned char>(legColorVec.x * 255),
                    static_cast<unsigned char>(legColorVec.y * 255),
                    static_cast<unsigned char>(legColorVec.z * 255), 255};

  getTorso().visual.setColor(torsoColor);
  getLeftArm().visual.setColor(armColor);
  getRightArm().visual.setColor(armColor);
  getLeftLeg().visual.setColor(legColor);
  getRightLeg().visual.setColor(legColor);
}

void NPC::changeState(std::unique_ptr<NPCState> newState) {
  if (currentState) {
    currentState->exit(this);
  }

  currentState = std::move(newState);

  if (currentState) {
    currentState->enter(this);
  }
}

void NPC::update(float deltaTime) {
  if (!isActive) {
    return;
  }

  HumanoidCharacter::update(deltaTime);

  updateLifetime(deltaTime);

  if (currentState) {
    currentState->update(this, deltaTime);
  }

  if (hasDestination) {
    followPath(deltaTime);
  }

  if (lifetimeTimer >= maxLifetime) {
    setActive(false);
    notifyExited();
  }
}

void NPC::moveTowards(Vector3 target, float deltaTime) {
  Vector3 currentPos = getPosition();
  Vector3 direction = Vector3Subtract(target, currentPos);
  float distanceToTarget = Vector3Length(direction);

  if (distanceToTarget > 0.3f) {
    direction = Vector3Normalize(direction);

    bool isCloseToTarget = distanceToTarget < 4.0f;
    bool isApproachingShopEntrance = false;

    if (targetShop) {
      Vector3 entrancePos = targetShop->getEntrancePosition();
      float distanceToEntrance = Vector3Distance(currentPos, entrancePos);
      isApproachingShopEntrance = distanceToEntrance < 6.0f;
    }

    if (!isCloseToTarget && !isApproachingShopEntrance &&
        wouldCollideAfterMovement(direction, deltaTime * 2.0f)) {
      float angle45 = PI / 4.0f;
      Vector3 leftDirection = {
          direction.x * cos(angle45) - direction.z * sin(angle45), direction.y,
          direction.x * sin(angle45) + direction.z * cos(angle45)};
      leftDirection = Vector3Normalize(leftDirection);

      if (!wouldCollideAfterMovement(leftDirection, deltaTime * 2.0f)) {
        direction = leftDirection;
      } else {
        Vector3 rightDirection = {
            direction.x * cos(-angle45) - direction.z * sin(-angle45),
            direction.y,
            direction.x * sin(-angle45) + direction.z * cos(-angle45)};
        rightDirection = Vector3Normalize(rightDirection);

        if (!wouldCollideAfterMovement(rightDirection, deltaTime * 2.0f)) {
          direction = rightDirection;
        }
      }
    }

    applyEnhancedMovementForces({direction.x, 0, direction.z}, movementSpeed,
                                5.0f);
  }
}

bool NPC::isNearTarget(Vector3 target, float threshold) const {
  return Vector3Distance(getPosition(), target) <= threshold;
}

void NPC::setDestination(Vector3 destination) {
  currentDestination = destination;
  hasDestination = true;
  pathWaypoints.clear();
  currentWaypointIndex = 0;

  if (navMesh) {
    std::vector<Vector3> path = navMesh->findPath(getPosition(), destination);
    if (!path.empty()) {
      pathWaypoints = path;
    } else {
      // Try to find a position closer to the target that is walkable
      Vector3 currentPos = getPosition();
      Vector3 direction = Vector3Subtract(destination, currentPos);
      direction = Vector3Normalize(direction);

      // Try intermediate positions along the direction
      for (float distance = 2.0f; distance <= 10.0f; distance += 2.0f) {
        Vector3 intermediatePos =
            Vector3Add(currentPos, Vector3Scale(direction, distance));
        if (navMesh->isWalkable(intermediatePos)) {
          std::vector<Vector3> partialPath =
              navMesh->findPath(getPosition(), intermediatePos);
          if (!partialPath.empty()) {
            pathWaypoints = partialPath;
            break;
          }
        }
      }
    }
  }
}

void NPC::followPath(float deltaTime) {
  if (!hasDestination) return;

  if (!pathWaypoints.empty() && currentWaypointIndex < pathWaypoints.size()) {
    Vector3 currentWaypoint = pathWaypoints[currentWaypointIndex];
    float distanceToWaypoint = Vector3Distance(getPosition(), currentWaypoint);

    float waypointThreshold = 1.0f;

    if (lastWaypointIndex != currentWaypointIndex) {
      waypointAttempts = 0;
      lastWaypointIndex = currentWaypointIndex;
      stuckTimer = 0.0f;
      hasTriedAlternative = false;
    }
    waypointAttempts++;

    float movementDistance = Vector3Distance(getPosition(), lastPosition);
    if (movementDistance < 0.03f) {  // Slightly less sensitive
      stuckTimer += deltaTime;
    } else {
      stuckTimer = 0.0f;
    }
    lastPosition = getPosition();

    if (stuckTimer > 3.0f && !hasTriedAlternative &&
        navMesh) {  // Increased patience
      std::vector<Vector3> alternativePath = navMesh->findAlternativePath(
          getPosition(), currentDestination, currentWaypoint);

      if (!alternativePath.empty() && alternativePath.size() > 2) {
        pathWaypoints = alternativePath;
        currentWaypointIndex = 0;
        hasTriedAlternative = true;
      } else {
        currentWaypointIndex++;
        hasTriedAlternative = true;
      }
      stuckTimer = 0.0f;
    }

    if (waypointAttempts > 300) {  // Increased patience
      waypointThreshold = 2.5f;
    }

    if (waypointAttempts > 600) {  // Increased patience
      currentWaypointIndex++;
      waypointAttempts = 0;
      hasTriedAlternative = false;
      stuckTimer = 0.0f;
    }

    if (distanceToWaypoint < waypointThreshold) {
      currentWaypointIndex++;
      waypointAttempts = 0;

      if (currentWaypointIndex < pathWaypoints.size()) {
      } else {
        hasDestination = false;
        pathWaypoints.clear();
        currentWaypointIndex = 0;
      }
    } else {
      moveTowards(currentWaypoint, deltaTime);
    }
  } else {
    moveTowards(currentDestination, deltaTime);
  }
}

void NPC::addObserver(NPCObserver* observer) {
  if (observer && std::find(observers.begin(), observers.end(), observer) ==
                      observers.end()) {
    observers.push_back(observer);
  }
}

void NPC::removeObserver(NPCObserver* observer) {
  observers.erase(std::remove(observers.begin(), observers.end(), observer),
                  observers.end());
}

void NPC::notifyFruitPicked(std::shared_ptr<Fruit> fruit) {
  for (auto* observer : observers) {
    observer->onNPCFruitPicked(this, fruit);
  }
}

void NPC::notifyExited() {
  for (auto* observer : observers) {
    observer->onNPCExited(this);
  }
}

void NPC::notifyEnteredShop() {
  for (auto* observer : observers) {
    observer->onNPCEnteredShop(this);
  }
}

std::shared_ptr<Fruit> NPC::findNearestFruit() const {
  if (!targetShop) {
    return nullptr;
  }

  return targetShop->findNearestFruit(getPosition());
}

Vector3 NPC::getRandomPositionInShop() const {
  if (!targetShop) {
    return getPosition();
  }

  return targetShop->getRandomInteriorPosition();
}

Vector3 NPC::getExitPosition() const {
  if (!targetShop) {
    return {20.0f, 0.5f, 20.0f};
  }

  Vector3 entrance = targetShop->getEntrancePosition();
  return {entrance.x, entrance.y, entrance.z + 5.0f};
}

bool NPC::isInsideShop() const {
  if (!targetShop) {
    return false;
  }

  return targetShop->isInsideShop(getPosition());
}

void NPC::interact() {
  // NPCs can be interacted with to show their state or inventory
  // For now, this is a placeholder
}

void NPC::sayMessage(const std::string& context) const {
  if (chatSystem && chatSystem->canSpeak()) {
    std::string message;

    if (context == "greeting") {
      message = chatSystem->getRandomGreeting();
    } else if (context == "shopping") {
      message = chatSystem->getRandomShoppingComment();
    } else if (context == "fruit") {
      message = chatSystem->getRandomFruitComment();
    } else if (context == "leaving") {
      message = chatSystem->getRandomLeavingComment();
    } else {
      message = chatSystem->getRandomGeneralComment();
    }

    if (!message.empty()) {
      chatSystem->addMessage(message, getPosition(), 3.0f, this);
    }
  } else {
    if (!chatSystem) {
    } else {
    }
  }
}

void NPC::updateLifetime(float deltaTime) { lifetimeTimer += deltaTime; }

Vector3 NPC::calculateRandomColor() const {
  static std::random_device rd;
  static std::mt19937 gen(rd());
  std::uniform_real_distribution<float> colorDist(0.3f, 1.0f);

  return {colorDist(gen), colorDist(gen), colorDist(gen)};
}
