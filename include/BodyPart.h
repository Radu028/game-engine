#pragma once

#include <string>

#include "objects/GameObject.h"
#include "raylib.h"

class GameObject;

class BodyPart : public GameObject {
 private:
  Vector3 size;
  Color color;
  Vector3 rotationAxis;
  float rotationAngle;

  Model* model;

  // Health-related members for academic requirements
  std::string name;
  float health;
  float maxHealth;
  bool isInjured;
  std::string injuryType;

 public:
  BodyPart(Vector3 position, Vector3 size, Color color,
           bool hasCollision = true);

  BodyPart(const BodyPart& other);

  BodyPart& operator=(const BodyPart& other);

  BodyPart(BodyPart&& other) noexcept;

  BodyPart& operator=(BodyPart&& other) noexcept;

  ~BodyPart() override;

  friend void swap(BodyPart& first, BodyPart& second) noexcept;

  void setRotation(const Vector3 axis, float angle);
  void setPosition(Vector3 newPos);
  void setColor(Color newColor);

  Vector3 getSize() const;
  Color getColor() const;

  float getRotationAngle() const;
  Vector3 getRotationAxis() const;

  void draw() const override;
  BoundingBox getBoundingBox() const override;
  std::unique_ptr<GameObject> clone() const override;

  // Health-related methods for academic requirements
  void setName(const std::string& partName) { name = partName; }
  void setHealth(float currentHealth, float maximum) {
    health = currentHealth;
    maxHealth = maximum;
  }
  const std::string& getName() const { return name; }
  float getHealth() const { return health; }
  float getMaxHealth() const { return maxHealth; }
  bool getIsInjured() const { return isInjured; }
  const std::string& getInjuryType() const { return injuryType; }

  void takeDamage(float damage);
  void heal(float amount);
  float getHealthPercentage() const;
  void displayStatus() const;

  void setBulletBody(btRigidBody* body) { GameObject::setBulletBody(body); }
  btRigidBody* getBulletBody() const { return GameObject::getBulletBody(); }
};
