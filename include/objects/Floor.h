#pragma once

#include <string>

#include "objects/StaticWorldObject.h"

class StaticWorldObject;

class Floor : public StaticWorldObject {
  Vector3 dimensions;
  Color color;

  bool hasTexture;
  Model model;

 public:
  Floor(Vector3 position, Vector3 dimensions, Color color,
        bool hasCollision = true);
  Floor(Vector3 position, Vector3 dimensions, std::string texturePath,
        bool hasCollision = true);

  void draw() const override;
  BoundingBox getBoundingBox() const override;
  std::unique_ptr<GameObject> clone() const override;
  PhysicsBodyConfig getPhysicsConfig() const override;
  void configurePhysicsBody(btRigidBody& body) const override;

  // Floor should not block pathfinding - it's the surface NPCs walk on
  std::string getObstacleType() const override { return "floor"; }
};