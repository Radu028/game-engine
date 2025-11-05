#pragma once

#include "objects/StaticWorldObject.h"

class Wall : public StaticWorldObject {
 private:
  Vector3 dimensions;
  Color color;

  Model model;
  bool useShaders;

 public:
  Wall(Vector3 position, Vector3 dimensions, Color color,
       bool hasCollisions = true, bool useShaders = true);

  void draw() const override;
  BoundingBox getBoundingBox() const override;
  std::unique_ptr<GameObject> clone() const override;
  PhysicsBodyConfig getPhysicsConfig() const override;
  void configurePhysicsBody(btRigidBody& body) const override;
};