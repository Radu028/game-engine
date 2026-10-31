#include "ai/NPC.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <utility>

#include "ai/NPCStates.h"
#include "raymath.h"

std::shared_ptr<NavMesh> NPC::navMesh = nullptr;

Vector3 NPC::getPosition() const {
  return character_ ? character_->getPosition() : Vector3{0.0f, 0.0f, 0.0f};
}

Vector3 NPC::getTorsoPosition() const {
  return character_ ? character_->getTorsoPosition()
                    : Vector3{0.0f, 0.0f, 0.0f};
}

Vector3 NPC::getFeetPosition() const {
  return character_ ? character_->getFeetPosition() : Vector3{0.0f, 0.0f, 0.0f};
}

BoundingBox NPC::getBoundingBox() const {
  return character_ ? character_->getBoundingBox()
                    : BoundingBox{{0, 0, 0}, {0, 0, 0}};
}

NPC::NPC(Vector3 position, std::shared_ptr<Shop> shop, GameWorld* world)
    : character_(std::make_shared<HumanoidCharacter>(position, world)),
      targetShop(std::move(shop)),
      chatSystem(nullptr),
      movementSpeed(3.0f),
      isActive(true),
      exitNotified(false),
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

  character_->getTorso().visual.setColor(torsoColor);
  character_->getLeftArm().visual.setColor(armColor);
  character_->getRightArm().visual.setColor(armColor);
  character_->getLeftLeg().visual.setColor(legColor);
  character_->getRightLeg().visual.setColor(legColor);
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

  updateLifetime(deltaTime);

  if (currentState) {
    currentState->update(this, deltaTime);
  }

  // States are allowed to deactivate the NPC (e.g., leaving the scene).
  if (!isActive) {
    return;
  }

  if (hasDestination) {
    followPath(deltaTime);
  }

  if (lifetimeTimer >= maxLifetime) {
    setActive(false);
  }
}

void NPC::moveTowards(Vector3 target, float deltaTime) {
  Vector3 currentPos = getPosition();
  Vector3 direction = Vector3Subtract(target, currentPos);
  float distanceToTarget = Vector3Length(direction);

  if (distanceToTarget > 0.3f) {
    direction = Vector3Normalize(direction);

    // Check for potential collision and try to find alternative direction
    float lookAheadDistance = std::max(1.0f, movementSpeed * deltaTime * 3.0f);

    if (wouldCollideAfterMovement(direction, lookAheadDistance)) {
      // Try multiple angles to find a clear path
      const float angles[] = {PI / 6.0f, -PI / 6.0f,   // 30 degrees
                              PI / 4.0f, -PI / 4.0f,   // 45 degrees
                              PI / 3.0f, -PI / 3.0f,   // 60 degrees
                              PI / 2.0f, -PI / 2.0f};  // 90 degrees

      bool foundClear = false;
      for (float angle : angles) {
        Vector3 testDir = {direction.x * cos(angle) - direction.z * sin(angle),
                           direction.y,
                           direction.x * sin(angle) + direction.z * cos(angle)};
        testDir = Vector3Normalize(testDir);

        if (!wouldCollideAfterMovement(testDir, lookAheadDistance)) {
          direction = testDir;
          foundClear = true;
          break;
        }
      }

      // If no clear direction found, try moving perpendicular to the wall
      if (!foundClear) {
        // Try to slide along the obstacle
        Vector3 slideDir = {-direction.z, 0.0f, direction.x};
        if (!wouldCollideAfterMovement(slideDir, lookAheadDistance * 0.5f)) {
          direction = slideDir;
        } else {
          slideDir = {direction.z, 0.0f, -direction.x};
          if (!wouldCollideAfterMovement(slideDir, lookAheadDistance * 0.5f)) {
            direction = slideDir;
          }
        }
      }
    }

    character_->applyMovement({direction.x, 0.0f, direction.z}, movementSpeed,
                              5.0f);
  }
}

bool NPC::isNearTarget(Vector3 target, float threshold) const {
  return Vector3Distance(getPosition(), target) <= threshold;
}

void NPC::setActive(bool active) {
  if (isActive == active) {
    return;
  }

  isActive = active;

  if (!isActive) {
    hasDestination = false;
    pathWaypoints.clear();
    currentWaypointIndex = 0;
    notifyExited();
  }
}

bool NPC::setDestination(Vector3 destination) {
  currentDestination = destination;
  pathWaypoints.clear();
  currentWaypointIndex = 0;
  waypointAttempts = 0;
  lastWaypointIndex = -1;
  stuckTimer = 0.0f;
  hasTriedAlternative = false;

  const Vector3 currentPos = getPosition();

  if (navMesh) {
    std::vector<Vector3> path = navMesh->findPath(currentPos, destination);

    if (path.empty()) {
      const auto& nodes = navMesh->getNodes();
      const int endNode = navMesh->findNearestNode(destination);
      if (endNode >= 0 && endNode < static_cast<int>(nodes.size())) {
        path = navMesh->findPath(
            currentPos, nodes[static_cast<size_t>(endNode)].position);
      }

      if (path.empty()) {
        const int startNode = navMesh->findNearestNode(currentPos);
        if (startNode >= 0 && endNode >= 0 &&
            startNode < static_cast<int>(nodes.size()) &&
            endNode < static_cast<int>(nodes.size())) {
          path = navMesh->findPath(
              nodes[static_cast<size_t>(startNode)].position,
              nodes[static_cast<size_t>(endNode)].position);
        }
      }
    }

    if (path.empty()) {
      Vector3 nearbyTarget = findNearestWalkablePosition(destination, 5.0f);
      if (Vector3Distance(nearbyTarget, currentPos) > 0.25f) {
        path = navMesh->findPath(currentPos, nearbyTarget);
      }
    }

    if (path.empty()) {
      hasDestination = false;
      return false;
    }

    while (path.size() > 1 &&
           Vector3Distance(currentPos, path.front()) < 0.45f) {
      path.erase(path.begin());
    }

    pathWaypoints = std::move(path);
    hasDestination = true;
    return true;
  } else {
    // No navigation mesh available: move directly.
    pathWaypoints.push_back(destination);
    hasDestination = true;
    return true;
  }
}

void NPC::followPath(float deltaTime) {
  if (!hasDestination) return;

  auto clearDestination = [&]() {
    hasDestination = false;
    pathWaypoints.clear();
    currentWaypointIndex = 0;
    waypointAttempts = 0;
    lastWaypointIndex = -1;
    stuckTimer = 0.0f;
    hasTriedAlternative = false;
  };

  if (pathWaypoints.empty() ||
      currentWaypointIndex >= static_cast<int>(pathWaypoints.size())) {
    if (Vector3Distance(getPosition(), currentDestination) < 1.5f) {
      clearDestination();
      return;
    }

    if (navMesh) {
      if (!setDestination(currentDestination)) {
        clearDestination();
      }
      return;
    }

    moveTowards(currentDestination, deltaTime);
    return;
  }

  while (currentWaypointIndex < static_cast<int>(pathWaypoints.size()) &&
         Vector3Distance(getPosition(), pathWaypoints[currentWaypointIndex]) <
             0.35f) {
    currentWaypointIndex++;
  }

  if (currentWaypointIndex >= static_cast<int>(pathWaypoints.size())) {
    if (Vector3Distance(getPosition(), currentDestination) < 1.5f) {
      clearDestination();
    } else if (navMesh) {
      setDestination(currentDestination);
    } else {
      moveTowards(currentDestination, deltaTime);
    }
    return;
  }

  Vector3 currentWaypoint = pathWaypoints[currentWaypointIndex];
  float distanceToWaypoint = Vector3Distance(getPosition(), currentWaypoint);
  float baseThreshold = 0.9f;
  float waypointThreshold = baseThreshold;

  if (lastWaypointIndex != currentWaypointIndex) {
    waypointAttempts = 0;
    lastWaypointIndex = currentWaypointIndex;
    stuckTimer = 0.0f;
    hasTriedAlternative = false;
  }
  waypointAttempts++;

  float movementDistance = Vector3Distance(getPosition(), lastPosition);
  if (movementDistance < 0.02f) {
    stuckTimer += deltaTime;
  } else {
    stuckTimer = std::max(0.0f, stuckTimer - deltaTime * 0.5f);
  }
  lastPosition = getPosition();

  if (stuckTimer > 2.0f && !hasTriedAlternative && navMesh) {
    std::vector<Vector3> alternativePath =
        navMesh->findAlternativePath(getPosition(), currentDestination,
                                     currentWaypoint);

    hasTriedAlternative = true;
    stuckTimer = 0.0f;

    if (!alternativePath.empty()) {
      pathWaypoints = std::move(alternativePath);
      currentWaypointIndex = 0;
    }
    return;
  }

  if (stuckTimer > 4.0f && navMesh) {
    if (setDestination(currentDestination)) {
      stuckTimer = 0.0f;
      hasTriedAlternative = false;
      return;
    }

    currentWaypointIndex = std::min(currentWaypointIndex + 1,
                                    static_cast<int>(pathWaypoints.size()) - 1);
    stuckTimer = 0.0f;
    hasTriedAlternative = false;
  }

  if (waypointAttempts > 160) {
    waypointThreshold = baseThreshold * 1.5f;
  }
  if (waypointAttempts > 320) {
    waypointThreshold = baseThreshold * 2.0f;
  }
  if (waypointAttempts > 500) {
    currentWaypointIndex++;
    waypointAttempts = 0;
    hasTriedAlternative = false;
    stuckTimer = 0.0f;
  }

  if (distanceToWaypoint < waypointThreshold) {
    currentWaypointIndex++;
    waypointAttempts = 0;
    stuckTimer = 0.0f;
    return;
  }

  moveTowards(currentWaypoint, deltaTime);
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
  if (exitNotified) return;
  exitNotified = true;
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

void NPC::sayMessage(const std::string& context) const {
  if (!chatSystem || !chatSystem->canSpeak()) {
    return;
  }

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
    chatSystem->addMessage(message, getTorsoPosition());
  }
}

void NPC::updateLifetime(float deltaTime) { lifetimeTimer += deltaTime; }

void NPC::removeFromPhysics(btDiscreteDynamicsWorld* world) {
  if (!character_) return;
  character_->removeFromPhysics(world);
}

bool NPC::wouldCollideAfterMovement(Vector3 direction, float distance) const {
  if (!character_) return false;
  return character_->wouldCollideAfterMovement(direction, distance);
}

float NPC::getVerticalCollisionContactTime(const Vector3& verticalMovement,
                                           const GameWorld* world,
                                           int maxIterations) const {
  if (!character_) return 0.0f;
  return character_->getVerticalCollisionContactTime(verticalMovement, world,
                                                     maxIterations);
}

Vector3 NPC::calculateRandomColor() const {
  static std::random_device rd;
  static std::mt19937 gen(rd());
  static const Color palette[] = {
      {0, 162, 232, 255},   // Classic Roblox bright blue
      {34, 177, 76, 255},   // Green
      {237, 28, 36, 255},   // Red
      {255, 127, 39, 255},  // Orange
      {255, 242, 0, 255},   // Yellow
      {163, 73, 164, 255},  // Purple
      {112, 146, 190, 255}  // Slate blue
  };
  std::uniform_int_distribution<size_t> indexDist(
      0, sizeof(palette) / sizeof(palette[0]) - 1);

  const Color picked = palette[indexDist(gen)];
  return {static_cast<float>(picked.r) / 255.0f,
          static_cast<float>(picked.g) / 255.0f,
          static_cast<float>(picked.b) / 255.0f};
}

Vector3 NPC::findNearestWalkablePosition(Vector3 center, float radius) const {
  if (!navMesh) {
    return center;
  }

  Vector3 bestPosition = center;
  float bestScore = -1.0f;

  // Try positions in a grid pattern around the center
  int gridSize = static_cast<int>(radius / 0.5f);  // 0.5m grid spacing

  for (int x = -gridSize; x <= gridSize; ++x) {
    for (int z = -gridSize; z <= gridSize; ++z) {
      Vector3 testPosition = {center.x + x * 0.5f, center.y,
                              center.z + z * 0.5f};

      float distance = Vector3Distance(testPosition, center);
      if (distance > radius) continue;

      if (navMesh->isWalkable(testPosition)) {
        // Score based on distance (closer is better) and how "open" the area is
        float distanceScore =
            (radius - distance) / radius;  // 0-1, higher is closer

        // Check how many nearby positions are also walkable (openness)
        int walkableNeighbors = 0;
        for (int dx = -1; dx <= 1; ++dx) {
          for (int dz = -1; dz <= 1; ++dz) {
            if (dx == 0 && dz == 0) continue;
            Vector3 neighborPos = {testPosition.x + dx * 0.5f, testPosition.y,
                                   testPosition.z + dz * 0.5f};
            if (navMesh->isWalkable(neighborPos)) {
              walkableNeighbors++;
            }
          }
        }

        float opennessScore =
            walkableNeighbors / 8.0f;  // 0-1, higher is more open
        float totalScore = distanceScore * 0.7f + opennessScore * 0.3f;

        if (totalScore > bestScore) {
          bestScore = totalScore;
          bestPosition = testPosition;
        }
      }
    }
  }

  return bestPosition;
}
