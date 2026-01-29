#pragma once

#include <memory>

#include "objects/GameObject.h"
#include "raylib.h"

// A lightweight render-only part used by HumanoidCharacter (no independent
// physics). Implemented as a GameObject for convenience (position + draw API).
class BodyPart final : public GameObject {
 public:
  BodyPart(Vector3 position, Vector3 size, Color color,
           bool hasCollision = false);
  ~BodyPart() override = default;

  BodyPart(const BodyPart&) = delete;
  BodyPart& operator=(const BodyPart&) = delete;
  BodyPart(BodyPart&&) = delete;
  BodyPart& operator=(BodyPart&&) = delete;

  void setRotation(Vector3 axis, float angleDegrees);
  void setColor(Color newColor) { color_ = newColor; }

  Vector3 getSize() const { return size_; }
  Color getColor() const { return color_; }
  float getRotationAngle() const { return rotationAngleDegrees_; }
  Vector3 getRotationAxis() const { return rotationAxis_; }

  void draw() const override;
  BoundingBox getBoundingBox() const override;
  std::unique_ptr<GameObject> clone() const override;

 private:
  Vector3 size_{};
  Color color_{WHITE};
  Vector3 rotationAxis_{0.0f, 1.0f, 0.0f};
  float rotationAngleDegrees_{0.0f};

  Model* model_ = nullptr;  // Shared unit cube model
};


