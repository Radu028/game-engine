#include "objects/GameObject.h"

#include <btBulletDynamicsCommon.h>

#include <cmath>

#include "GameWorld.h"
#include "raymath.h"

GameObject::GameObject(Vector3 position, bool hasCollision,
                       bool affectedByGravity, bool isStatic)
    : position(position),
      velocity({0.0f, 0.0f, 0.0f}),
      isOnGround(false),
      affectedByGravity(affectedByGravity),
      isStatic(isStatic),
      hasCollision(hasCollision),
      bulletBody(nullptr) {}

GameObject::~GameObject() {}

bool GameObject::checkCollision(const GameObject& other) const {
  if (!this->hasCollision || !other.getHasCollision()) {
    return false;
  }
  return CheckCollisionBoxes(this->getBoundingBox(), other.getBoundingBox());
}

bool GameObject::checkCollisionWith(const BoundingBox& otherBox) const {
  if (!hasCollision) return false;
  return CheckCollisionBoxes(getBoundingBox(), otherBox);
}

float GameObject::getDistance(const GameObject& other) const {
  float dx = other.position.x - this->position.x;
  float dy = other.position.y - this->position.y;
  float dz = other.position.z - this->position.z;
  return sqrtf(dx * dx + dy * dy + dz * dz);
}

float GameObject::getDistanceSquared(const GameObject& other) const {
  float dx = other.position.x - this->position.x;
  float dy = other.position.y - this->position.y;
  float dz = other.position.z - this->position.z;
  return dx * dx + dy * dy + dz * dz;
}

float GameObject::getVerticalCollisionContactTime(
    const Vector3& verticalMovementVector, const GameWorld* world,
    int maxIterations) const {
  if (!world ||
      (verticalMovementVector.x == 0 && verticalMovementVector.y == 0 &&
       verticalMovementVector.z == 0)) {
    return 1.0f;
  }

  Vector3 originalPosition = getPosition();
  float t0 = 0.0f;
  float t1 = 1.0f;
  float tMid;

  auto checkCollisionAtDisplacement = [&](const Vector3& displacement) -> bool {
    BoundingBox objOriginalBox = getBoundingBox();
    BoundingBox objTestBox = {Vector3Add(objOriginalBox.min, displacement),
                              Vector3Add(objOriginalBox.max, displacement)};

    for (const auto& otherSharedPtr : world->getObjects()) {
      const GameObject* other = otherSharedPtr.get();
      if (!other || other == this || !other->getHasCollision() ||
          other->getIsStatic() == this->getIsStatic()) {
        continue;
      }

      if (CheckCollisionBoxes(objTestBox, other->getBoundingBox())) {
        return true;
      }
    }

    return false;
  };

  Vector3 fullDisplacement = verticalMovementVector;

  if (!checkCollisionAtDisplacement(fullDisplacement)) {
    return 1.0f;
  }

  for (int i = 0; i < maxIterations; i++) {
    if ((t1 - t0) < EPSILON) break;
    tMid = (t0 + t1) / 2.0f;
    Vector3 testDisplacementAtMid = Vector3Scale(verticalMovementVector, tMid);

    if (checkCollisionAtDisplacement(testDisplacementAtMid)) {
      t1 = tMid;
    } else {
      t0 = tMid;
    }
  }

  return t0;
}
