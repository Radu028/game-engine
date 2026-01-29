#pragma once

#include "objects/StaticWorldObject.h"

class Wall : public StaticWorldObject {
 private:
  Vector3 dimensions;
  Color color;

  Model model;

 public:
  Wall(Vector3 position, Vector3 dimensions, Color color,
       bool hasCollisions = true);
  ~Wall() override;

  Wall(const Wall&) = delete;
  Wall& operator=(const Wall&) = delete;
  Wall(Wall&&) = delete;
  Wall& operator=(Wall&&) = delete;

  void draw() const override;
  BoundingBox getBoundingBox() const override;
  std::unique_ptr<GameObject> clone() const override;
  PhysicsBodyConfig getPhysicsConfig() const override;
  void configurePhysicsBody(btRigidBody& body) const override;
};