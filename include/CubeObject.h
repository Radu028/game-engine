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

  //  protected:
  //   float width, height, length;
  Color color;

  //   Texture2D texture;
  //   bool hasTexture;
  //   Model model;

 public:
  CubeObject(const Vector3& position, float width, float height, float depth,
             Color color, bool isStatic, const std::string& texturePath = "");
  ~CubeObject();

  BoundingBox getBoundingBox() const;
  void draw() const override;

  bool checkCollision(const CubeObject& other) const;
};

#endif