#ifndef PLAYER_H
#define PLAYER_H

#include <BulletDynamics/Character/btKinematicCharacterController.h>
#include <btBulletDynamicsCommon.h>

#include <iostream>
#include <memory>
#include <string>

#include "CubeObject.h"
#include "GameObject.h"
#include "raylib.h"

class GameWorld;

enum Direction { FORWARD, BACKWARD, LEFT, RIGHT };

class Player : public GameObject {
 private:
  Vector3 velocity;
  bool isOnGround;
  GameWorld* world;

  std::unique_ptr<CubeObject> head;
  std::unique_ptr<CubeObject> torso;
  std::unique_ptr<CubeObject> leftArm;
  std::unique_ptr<CubeObject> rightArm;
  std::unique_ptr<CubeObject> leftLeg;
  std::unique_ptr<CubeObject> rightLeg;
  bool isMoving;

  btConvexShape* characterShape;
  btPairCachingGhostObject* ghostObject;
  btKinematicCharacterController* characterController;

  void move(Direction direction, float byValue);
  void jump();
  float findMaxSafePosition(float start, float end, float* positionComponent);
  void moveWithSliding(float start, float end, float* positionComponent);

 public:
  Player(Vector3 startPosition = (Vector3){0.0f, 0.0f, 0.0f});
  ~Player();

  void draw() const override;

  void setWorld(GameWorld* gameWorld) { this->world = gameWorld; }

  void handleInput(float movementSpeed);

  void applyGravity(float deltaTime);

  void update(float deltaTime) override;
  bool checkCollisionWithWorld() const;
};

#endif