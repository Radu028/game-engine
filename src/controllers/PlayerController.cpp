#include "controllers/PlayerController.h"

#include <cmath>

#include "raylib.h"
#include "raymath.h"

PlayerController::PlayerController(
    std::shared_ptr<HumanoidCharacter> character)
    : character_(std::move(character)) {}

void PlayerController::update(float cameraAngleX,
                              float movementSpeed) const {
  if (!character_) return;

  Vector2 inputAxis = readMovementAxis();
  Vector3 worldDirection = cameraAlignedDirection(inputAxis, cameraAngleX);

  if (Vector3Length(worldDirection) > 0.1f) {
    character_->applyMovement(worldDirection, movementSpeed);
  } else {
    character_->applyMovement({0.0f, 0.0f, 0.0f}, 0.0f);
  }

  handleJump();
}

Vector2 PlayerController::readMovementAxis() const {
  Vector2 axis{0.0f, 0.0f};

  if (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP)) axis.y += 1.0f;
  if (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN)) axis.y -= 1.0f;
  if (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT)) axis.x += 1.0f;
  if (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT)) axis.x -= 1.0f;

  if (Vector2Length(axis) > 1.0f) axis = Vector2Normalize(axis);

  return axis;
}

Vector3 PlayerController::cameraAlignedDirection(const Vector2& axis,
                                                 float cameraAngleX) const {
  if (Vector2Length(axis) < 0.1f) return {0.0f, 0.0f, 0.0f};

  float cameraRad = cameraAngleX * DEG2RAD;

  const float forwardX = -sinf(cameraRad);
  const float forwardZ = -cosf(cameraRad);
  const float rightX = cosf(cameraRad);
  const float rightZ = -sinf(cameraRad);

  Vector3 direction{
      axis.y * forwardX + axis.x * rightX,
      0.0f,
      axis.y * forwardZ + axis.x * rightZ,
  };

  return Vector3Normalize(direction);
}

void PlayerController::handleJump() const {
  if (!character_) return;
  if (IsKeyPressed(KEY_SPACE)) character_->jump();
}


