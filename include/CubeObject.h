#ifndef CUBEOBJECT_H
#define CUBEOBJECT_H

#include <btBulletDynamicsCommon.h>

#include "GameObject.h"

class GameObject;

class CubeObject : public GameObject {
 private:
  btCollisionShape* collisionShape;
  btDefaultMotionState* motionState;
  btRigidBody* rigidBody;
  // Dimensions for rendering and bounding box
  float width;
  float height;
  float depth;
  Color color;

  //  protected:
  //   float width, height, length;
  //   Texture2D texture;
  //   bool hasTexture;
  //   Model model;

 public:
  // Synchronize cube position with physics simulation
  void update(float deltaTime) override;
  CubeObject(const Vector3& position, float width, float height, float depth,
             Color color, bool isStatic, const std::string& texturePath = "");
  ~CubeObject();

  BoundingBox getBoundingBox() const;
  void draw() const override;

  bool checkCollision(const CubeObject& other) const;
};

#endif