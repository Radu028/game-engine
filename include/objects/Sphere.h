#pragma once

#include "objects/StaticWorldObject.h"

class Sphere : public StaticWorldObject {
 private:
  float radius;
  Color color;

  Model model;

 public:
  Sphere(Vector3 position, float radius, Color color,
         bool hasCollisions = true);

  void draw() const override;
  BoundingBox getBoundingBox() const override;
  std::unique_ptr<GameObject> clone() const override;
  PhysicsBodyConfig getPhysicsConfig() const override;
  void configurePhysicsBody(btRigidBody& body) const override;
  float getRadius() const { return radius; }
};