#ifndef GAMEOBJECT_H
#define GAMEOBJECT_H

#include <btBulletDynamicsCommon.h>

#include <memory>  // For std::shared_ptr

#include "GameWorld.h"
#include "raylib.h"

class GameWorld;

class GameObject {
 protected:
  Vector3 position;
  Vector3 velocity;
  bool isOnGround;
  bool affectedByGravity;
  bool isStatic;
  bool hasCollision;
  btRigidBody* bulletBody = nullptr;  // Bullet physics rigid body pointer

 public:
  GameObject(Vector3 position, bool hasCollision = true,
             bool affectedByGravity = true, bool isStatic = false);
  virtual ~GameObject();

  Vector3 getPosition() const { return position; }
  Vector3 getVelocity() const { return velocity; }
  bool getIsOnGround() const { return isOnGround; }
  bool isAffectedByGravity() const { return affectedByGravity; }
  bool getIsStatic() const { return isStatic; }
  bool getHasCollision() const { return hasCollision; }
  void setBulletBody(btRigidBody* body) { bulletBody = body; }
  btRigidBody* getBulletBody() const { return bulletBody; }

  void setPosition(Vector3 newPosition) { position = newPosition; }
  void setVelocity(Vector3 newVelocity) { velocity = newVelocity; }
  void setIsOnGround(bool onGround) { isOnGround = onGround; }
  void setHasCollision(bool collision) { hasCollision = collision; }

  virtual void draw() const = 0;
  virtual BoundingBox getBoundingBox() const = 0;
  virtual std::unique_ptr<GameObject> clone() const = 0;

  virtual std::string getObstacleType() const { return "generic"; }
  virtual Vector3 getObstacleSize() const {
    try {
      auto bbox = getBoundingBox();
      return {bbox.max.x - bbox.min.x, bbox.max.y - bbox.min.y,
              bbox.max.z - bbox.min.z};
    } catch (...) {
      return {2.0f, 2.0f, 2.0f};  // Default size
    }
  }

  virtual void update(float deltaTime) {};
  virtual void interact() {};
  virtual bool checkCollisionWith(const BoundingBox& otherBox) const;
  virtual float getVerticalCollisionContactTime(
      const Vector3& verticalMovementVector, const GameWorld* world,
      int maxIterations) const;

  bool checkCollision(const GameObject& other) const;
  float getDistance(const GameObject& other) const;
  float getDistanceSquared(const GameObject& other) const;
};

#endif