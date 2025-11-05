#pragma once

class GameObject;
class btDiscreteDynamicsWorld;

class PhysicsEngine {
 public:
  virtual ~PhysicsEngine() = default;

  virtual void addObject(GameObject& object) = 0;
  virtual void removeObject(GameObject& object) = 0;
  virtual void update(float deltaTime) = 0;
  virtual btDiscreteDynamicsWorld* getNativeWorld() = 0;
};


