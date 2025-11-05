#pragma once

#include "raylib.h"

enum class ColliderType {
  None,
  Box,
  Sphere
};

struct PhysicsBodyConfig {
  bool usesPhysics = false;
  ColliderType collider = ColliderType::None;
  Vector3 dimensions = {0.0f, 0.0f, 0.0f};
  float radius = 0.0f;
  bool isStatic = true;
  bool affectedByGravity = false;
};


