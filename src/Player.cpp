#include "Player.h"

#include <cmath>

#include "CubeObject.h"
#include "GameWorld.h"
#include "Physics.h"
#include "PhysicsEngine.h"
#include "raylib.h"

Player::Player(Vector3 position) : GameObject(position, true) {
  head = std::make_unique<CubeObject>((Vector3){0.0f, 0.0f, 0.0f}, 0.8f, 0.8f,
                                      0.8f, BLUE, false);
  torso = std::make_unique<CubeObject>((Vector3){0.0f, 0.0f, 0.0f}, 1.0f, 1.2f,
                                       0.5f, BLUE, false);
  leftArm = std::make_unique<CubeObject>((Vector3){0.0f, 0.0f, 0.0f}, 0.3f,
                                         1.0f, 0.3f, BLUE, false);
  rightArm = std::make_unique<CubeObject>((Vector3){0.0f, 0.0f, 0.0f}, 0.3f,
                                          1.0f, 0.3f, BLUE, false);
  leftLeg = std::make_unique<CubeObject>((Vector3){0.0f, 0.0f, 0.0f}, 0.4f,
                                         1.0f, 0.4f, BLUE, false);
  rightLeg = std::make_unique<CubeObject>((Vector3){0.0f, 0.0f, 0.0f}, 0.4f,
                                          1.0f, 0.4f, BLUE, false);

  isMoving = false;

  characterShape = new btCapsuleShape(0.4f, 1.0f);

  ghostObject = new btPairCachingGhostObject();
  btTransform startTrans;
  startTrans.setIdentity();
  startTrans.setOrigin(btVector3(position.x, position.y, position.z));
  ghostObject->setWorldTransform(startTrans);
  ghostObject->setCollisionShape(characterShape);
  ghostObject->setCollisionFlags(ghostObject->getCollisionFlags() |
                                 btCollisionObject::CF_CHARACTER_OBJECT);

  characterController =
      new btKinematicCharacterController(ghostObject, characterShape, 0.35f);

  auto world = PhysicsEngine::getInstance().getDynamicsWorld();
  world->addCollisionObject(
      ghostObject, btBroadphaseProxy::CharacterFilter,
      btBroadphaseProxy::StaticFilter | btBroadphaseProxy::DefaultFilter);
  world->addAction(characterController);
}

Player::~Player() {
  auto world = PhysicsEngine::getInstance().getDynamicsWorld();
  world->removeAction(characterController);
  world->removeCollisionObject(ghostObject);
  delete characterController;
  delete ghostObject;
  delete characterShape;
}

void Player::handleInput(float movementSpeed) {
  if (IsKeyDown(KEY_W)) move(FORWARD, movementSpeed);
  if (IsKeyDown(KEY_S)) move(BACKWARD, movementSpeed);
  if (IsKeyDown(KEY_A)) move(LEFT, movementSpeed);
  if (IsKeyDown(KEY_D)) move(RIGHT, movementSpeed);
  if (IsKeyPressed(KEY_SPACE)) jump();
}

// void Player::move(Direction direction, float byValue) {
//   if (!world) return;
//   Vector3 oldPosition = position;

//   Vector3 newPosition = position;
//   if (direction == FORWARD) {
//     newPosition.z -= byValue;
//   } else if (direction == BACKWARD) {
//     newPosition.z += byValue;
//   } else if (direction == LEFT) {
//     newPosition.x -= byValue;
//   } else if (direction == RIGHT) {
//     newPosition.x += byValue;
//   }

//   // Move along X axis with sliding collision
//   if (newPosition.x != oldPosition.x) {
//     moveWithSliding(oldPosition.x, newPosition.x, &position.x);
//   }

//   // Move along Z axis with sliding collision
//   if (newPosition.z != oldPosition.z) {
//     moveWithSliding(oldPosition.z, newPosition.z, &position.z);
//   }
// }

void Player::jump() {
  if (!isOnGround) return;
  velocity.y = PhysicsSettings::JUMP_VELOCITY;
  isOnGround = false;
}

bool Player::checkCollisionWithWorld() const {
  for (const auto& obj : world->getObjects()) {
    if (torso->checkCollision(*obj)) {
      return true;
    }
  }
  return false;
}

float Player::findMaxSafePosition(float start, float end,
                                  float* positionComponent) {
  const int MAX_ITERATIONS = 10;  // Maximum number of binary search iterations
  float moveFactor = 0.0f;        // How much of the full movement to apply
  float step = 0.5f;              // Binary search step size
  float originalPosition = *positionComponent;

  // Binary search for maximum safe distance
  for (int i = 0; i < MAX_ITERATIONS; i++) {
    float testPosition = start + (end - start) * (moveFactor + step);

    // Try the test position
    *positionComponent = testPosition;
    bool hasCollision = checkCollisionWithWorld();

    if (hasCollision) {
      // Too far, try a smaller step
      step *= 0.5f;
    } else {
      // Can move further, increase moveFactor
      moveFactor += step;
      step *= 0.5f;
    }
  }

  // Reset position component before returning
  *positionComponent = originalPosition;
  return moveFactor;
}

void Player::moveWithSliding(float start, float end, float* positionComponent) {
  float originalValue = *positionComponent;

  // Try full movement first
  *positionComponent = end;

  // Check if we have a collision
  if (checkCollisionWithWorld()) {
    // Reset position and find maximum safe position
    *positionComponent = start;
    float moveFactor = findMaxSafePosition(start, end, positionComponent);

    // Apply the final safe position
    *positionComponent = start + (end - start) * moveFactor;
  }
}

void Player::applyGravity(float deltaTime) {
  if (!world) return;

  float clampedDelta = deltaTime;
  if (clampedDelta < PhysicsSettings::MIN_DELTA_TIME)
    clampedDelta = PhysicsSettings::MIN_DELTA_TIME;
  if (clampedDelta > PhysicsSettings::MAX_DELTA_TIME)
    clampedDelta = PhysicsSettings::MAX_DELTA_TIME;

  float scaledDelta = clampedDelta * PhysicsSettings::TIME_SCALE;
  velocity.y += PhysicsSettings::GRAVITY_ACCELERATION * scaledDelta;

  if (velocity.y < PhysicsSettings::MAX_FALL_SPEED) {
    velocity.y = PhysicsSettings::MAX_FALL_SPEED;
  }

  float initialY = position.y;
  float targetY = initialY + velocity.y * scaledDelta;

  position.y = targetY;
  bool collisionDetected = checkCollisionWithWorld();
  position.y = initialY;

  if (!collisionDetected) {
    position.y = targetY;
    return;
  }

  float t0 = 0.0f;
  float t1 = 1.0f;
  float tMid;

  const int MAX_ITERATIONS = 32;
  const float EPSILON = 0.001f;

  for (int i = 0; i < MAX_ITERATIONS && (t1 - t0) > EPSILON; i++) {
    tMid = (t0 + t1) / 2.0f;

    position.y = initialY + (targetY - initialY) * tMid;

    if (checkCollisionWithWorld()) {
      t1 = tMid;
    } else {
      t0 = tMid;
    }
  }

  position.y = initialY + (targetY - initialY) * t0;
  velocity.y = 0;
}

void Player::update(float deltaTime) {
  if (!world) return;

  PhysicsEngine::getInstance().stepSimulation(deltaTime);

  btTransform trans;
  ghostObject->getWorldTransform(trans);
  btVector3 org = trans.getOrigin();
  position = {org.getX(), org.getY, org.getZ()};

  GameObject::draw();

  applyGravity(deltaTime);

  // Calculate dimensions for ground check
  // float halfWidth = width / 2.0f;
  // float halfLength = length / 2.0f;
  // float feetY = position.y - height / 2.0f - 0.01f;
  BoundingBox torsoBox = torso->getBoundingBox();
  float halfWidth = torsoBox.max.x - position.x;
  float halfLenght = torsoBox.max.z - position.z;
  float feetY = torsoBox.min.y - 0.01f;

  // Define a grid of points to check on the player's base
  const int GRID_SIZE = 3;  // 3x3 grid (9 puncte)
  bool onObject = false;

  for (const auto& obj : world->getObjects()) {
    BoundingBox objectBox = obj->getBoundingBox();
    float objectTopY = objectBox.max.y;

    // Check grid of points across the entire base of the player
    for (int i = 0; i < GRID_SIZE; i++) {
      for (int j = 0; j < GRID_SIZE; j++) {
        // Calculate position for this grid point (evenly distributed)
        // float xOffset = -halfWidth + (width * i / (GRID_SIZE - 1));
        // float zOffset = -halfLength + (length * j / (GRID_SIZE - 1));
        float xOffset = -halfWidth + (2.0f * halfWidth) * i / (GRID_SIZE - 1);
        float zOffset = -halfLenght + (2.0f * halfLenght) * j / (GRID_SIZE - 1);

        Vector3 checkPoint = {position.x + xOffset, feetY,
                              position.z + zOffset};

        if (checkPoint.y <= objectTopY + 0.01f &&
            checkPoint.x >= objectBox.min.x &&
            checkPoint.x <= objectBox.max.x &&
            checkPoint.z >= objectBox.min.z &&
            checkPoint.z <= objectBox.max.z) {
          onObject = true;
          break;
        }
      }
      if (onObject) break;
    }
    if (onObject) break;
  }

  isOnGround = onObject;

  if (onObject && velocity.y <= 0) {
    velocity.y = 0;
  }

  float speedXZ = sqrt(velocity.x * velocity.x + velocity.z * velocity.z);
  isMoving = (speedXZ > 0.1f);

  float headOffsetY = 1.8f;
  float torsoOffsetY = 0.9f;
  float swing = isMoving ? sin(GetTime() * 10.0f) * 0.5f : 0.0f;

  head->setPosition(
      (Vector3){position.x, position.y + headOffsetY, position.z});
  torso->setPosition(
      (Vector3){position.x, position.y + torsoOffsetY, position.z});
  leftArm->setPosition((Vector3){
      position.x - 0.75f, position.y + torsoOffsetY + swing, position.z});
  rightArm->setPosition((Vector3){
      position.x + 0.75f, position.y + torsoOffsetY - swing, position.z});
  leftLeg->setPosition(
      (Vector3){position.x - 0.3f, position.y - 0.5f - swing, position.z});
  rightLeg->setPosition(
      (Vector3){position.x + 0.3f, position.y - 0.5f + swing, position.z});
}

void Player::draw() const {
  torso->draw();
  head->draw();
  leftArm->draw();
  rightArm->draw();
  leftLeg->draw();
  rightLeg->draw();
}