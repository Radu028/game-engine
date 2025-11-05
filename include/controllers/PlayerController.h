#pragma once

#include <memory>

#include "entities/HumanoidCharacter.h"

class PlayerController {
 public:
  explicit PlayerController(std::shared_ptr<HumanoidCharacter> character);

  void update(float cameraAngleX, float movementSpeed) const;

  std::shared_ptr<HumanoidCharacter> getCharacter() const { return character_; }

 private:
  std::shared_ptr<HumanoidCharacter> character_;

  Vector2 readMovementAxis() const;
  Vector3 cameraAlignedDirection(const Vector2& axis, float cameraAngleX) const;
  void handleJump() const;
};


