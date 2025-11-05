#pragma once

#include <memory>
#include <string>

#include "objects/GameObject.h"
#include "raylib.h"

class GameObject;

class CubeObject : public GameObject {
 protected:
  Vector3 size;
  Color color;

  Texture2D texture;
  bool hasTexture;
  bool textureLoaded;
  Model model;
  bool useShaders;

 public:
  CubeObject(Vector3 position, Vector3 size, Color color, bool hasCollision,
             const std::string& texturePath = "",
             bool affectedByGravity = false, bool isStatic = true,
             bool useShaders = true);
  ~CubeObject() override;

  CubeObject(const CubeObject& other);

  Vector3 getSize() const;

  BoundingBox getBoundingBox() const override;
  void draw() const override;
  std::unique_ptr<GameObject> clone() const override;

  bool checkCollision(const CubeObject& other) const;

  std::string getObstacleType() const override {
    return isStatic ? "wall" : "generic";
  }

  void interact() override;

  PhysicsBodyConfig getPhysicsConfig() const override;
  void configurePhysicsBody(btRigidBody& body) const override;

  void setUseShaders(bool use) { useShaders = use; }
};