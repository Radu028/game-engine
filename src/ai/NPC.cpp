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
  return character_ ? character_->getFeetPosition()
                    : Vector3{0.0f, 0.0f, 0.0f};
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

    if (!isCloseToTarget &&
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

    character_->applyMovement({direction.x, 0.0f, direction.z}, movementSpeed,
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
      // Enhanced fallback strategy for failed pathfinding
      Vector3 currentPos = getPosition();

      // Strategy 1: Try intermediate positions toward the target
      Vector3 direction = Vector3Subtract(destination, currentPos);
      float totalDistance = Vector3Length(direction);

      if (totalDistance > 0.1f) {
        direction = Vector3Normalize(direction);

        // Try positions at 25%, 50%, and 75% of the way to destination
        std::vector<float> distances = {
            totalDistance * 0.25f, totalDistance * 0.5f, totalDistance * 0.75f};

        for (float distance : distances) {
          Vector3 intermediatePos =
              Vector3Add(currentPos, Vector3Scale(direction, distance));
          if (navMesh->isWalkable(intermediatePos)) {
            std::vector<Vector3> partialPath =
                navMesh->findPath(currentPos, intermediatePos);
            if (!partialPath.empty()) {
              pathWaypoints = partialPath;
              break;  // Use first successful partial path
            }
          }
        }
      }

      // Strategy 2: If still no path, try nearby accessible areas
      if (pathWaypoints.empty()) {
        // Find the nearest walkable position to our current location
        Vector3 nearestWalkable = findNearestWalkablePosition(currentPos, 5.0f);
        if (Vector3Distance(nearestWalkable, currentPos) > 0.5f) {
          std::vector<Vector3> escapeePath =
              navMesh->findPath(currentPos, nearestWalkable);
          if (!escapeePath.empty()) {
            pathWaypoints = escapeePath;
          }
        }
      }

      // Strategy 3: Emergency movement - direct movement without pathfinding
      if (pathWaypoints.empty()) {
        // Create a simple straight-line path with a few waypoints
        Vector3 midPoint = Vector3Add(
            currentPos, Vector3Scale(direction, totalDistance * 0.5f));
        midPoint.y = 0.5f;  // Ensure ground level

        pathWaypoints.push_back(currentPos);
        pathWaypoints.push_back(midPoint);
        pathWaypoints.push_back(destination);
      }
    }
  } else {
    // No navigation mesh available - create direct path
    pathWaypoints.push_back(getPosition());
    pathWaypoints.push_back(destination);
  }
}

void NPC::followPath(float deltaTime) {
  if (!hasDestination) return;

  if (!pathWaypoints.empty() && currentWaypointIndex < pathWaypoints.size()) {
    Vector3 currentWaypoint = pathWaypoints[currentWaypointIndex];
    float distanceToWaypoint = Vector3Distance(getPosition(), currentWaypoint);

    // Dynamic waypoint threshold based on movement speed and situation
    float baseThreshold = 1.0f;
    float waypointThreshold = baseThreshold;

    // Reset tracking variables when moving to a new waypoint
    if (lastWaypointIndex != currentWaypointIndex) {
      waypointAttempts = 0;
      lastWaypointIndex = currentWaypointIndex;
      stuckTimer = 0.0f;
      hasTriedAlternative = false;
    }
    waypointAttempts++;

    // Enhanced stuck detection
    float movementDistance = Vector3Distance(getPosition(), lastPosition);
    bool isMoving =
        movementDistance > 0.02f;  // More sensitive movement detection

    if (!isMoving) {
      stuckTimer += deltaTime;
    } else {
      stuckTimer = std::max(
          0.0f,
          stuckTimer -
              deltaTime * 0.5f);  // Gradually reduce stuck timer when moving
    }
    lastPosition = getPosition();

    // Progressive response to being stuck
    if (stuckTimer > 2.0f && !hasTriedAlternative && navMesh) {
      // First attempt: Try alternative path
      std::vector<Vector3> alternativePath = navMesh->findAlternativePath(
          getPosition(), currentDestination, currentWaypoint);

      if (!alternativePath.empty() && alternativePath.size() > 1) {
        pathWaypoints = alternativePath;
        currentWaypointIndex = 0;
        hasTriedAlternative = true;
        stuckTimer = 0.0f;
      } else {
        // Second attempt: Skip to next waypoint
        currentWaypointIndex++;
        hasTriedAlternative = true;
        stuckTimer = 0.0f;
      }
    } else if (stuckTimer > 4.0f) {
      // Third attempt: Emergency unstuck procedure
      Vector3 unstuckPosition =
          findNearestWalkablePosition(getPosition(), 3.0f);
      if (Vector3Distance(unstuckPosition, getPosition()) > 0.5f) {
        // Move to unstuck position first
        setDestination(unstuckPosition);
        return;
      } else {
        // Force skip multiple waypoints
        currentWaypointIndex =
            std::min(currentWaypointIndex + 2,
                     static_cast<int>(pathWaypoints.size()) - 1);
        stuckTimer = 0.0f;
        hasTriedAlternative = false;
      }
    }

    // Progressive waypoint threshold increase for difficult waypoints
    if (waypointAttempts > 200) {  // Reduced from 300
      waypointThreshold = baseThreshold * 1.5f;
    }
    if (waypointAttempts > 400) {  // Reduced from 600
      waypointThreshold = baseThreshold * 2.0f;
    }
    if (waypointAttempts > 600) {  // Force skip after many attempts
      currentWaypointIndex++;
      waypointAttempts = 0;
      hasTriedAlternative = false;
      stuckTimer = 0.0f;
    }

    // Check if we've reached the current waypoint
    if (distanceToWaypoint < waypointThreshold) {
      currentWaypointIndex++;
      waypointAttempts = 0;
      stuckTimer = 0.0f;

      if (currentWaypointIndex >= pathWaypoints.size()) {
        // Reached destination
        hasDestination = false;
        pathWaypoints.clear();
        currentWaypointIndex = 0;
      }
    } else {
      // Move towards current waypoint
      moveTowards(currentWaypoint, deltaTime);
    }
  } else {
    // No waypoints available - direct movement to destination
    float distanceToDestination =
        Vector3Distance(getPosition(), currentDestination);

    if (distanceToDestination < 1.5f) {
      // Close enough to destination
      hasDestination = false;
      pathWaypoints.clear();
      currentWaypointIndex = 0;
    } else {
      moveTowards(currentDestination, deltaTime);
    }
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
    // chatSystem->addMessage(message, getPosition());
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
  std::uniform_real_distribution<float> colorDist(0.3f, 1.0f);

  return {colorDist(gen), colorDist(gen), colorDist(gen)};
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
