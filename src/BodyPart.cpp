#include "BodyPart.h"

#include <algorithm>
#include <iostream>

#include "raylib.h"
#include "systems/ShaderSystem.h"

static Model MakeSharedModel() {
  Mesh mesh = GenMeshCube(1.0f, 1.0f, 1.0f);
  Model model = LoadModelFromMesh(mesh);

  if (ShaderSystem* shaderSystem = ShaderSystem::getInstance()) {
    model.materials[0].shader = shaderSystem->getShader();
  }

  return model;
}

static Model& GetSharedModel() {
  static Model sharedModel = MakeSharedModel();
  return sharedModel;
}

BodyPart::BodyPart(Vector3 position, Vector3 size, Color color,
                   bool hasCollision)
    : GameObject(position, hasCollision, false, false),
      size(size),
      color(color),
      rotationAxis({0.0f, 1.0f, 0.0f}),
      rotationAngle(0.0f),
      model(&GetSharedModel()),
      name("Body Part"),
      health(100.0f),
      maxHealth(100.0f),
      isInjured(false),
      injuryType("") {}

BodyPart::BodyPart(const BodyPart& other)
    : GameObject(other),
      size(other.size),
      color(other.color),
      rotationAxis(other.rotationAxis),
      rotationAngle(other.rotationAngle),
      model(other.model),
      name(other.name),
      health(other.health),
      maxHealth(other.maxHealth),
      isInjured(other.isInjured),
      injuryType(other.injuryType) {
  std::cout << "BodyPart copy constructor called for: " << name << std::endl;
}

BodyPart& BodyPart::operator=(const BodyPart& other) {
  std::cout << "BodyPart copy assignment operator called" << std::endl;
  if (this != &other) {
    BodyPart temp(other);
    swap(*this, temp);
  }
  return *this;
}

BodyPart::BodyPart(BodyPart&& other) noexcept
    : GameObject(std::move(other)),
      size(other.size),
      color(other.color),
      rotationAxis(other.rotationAxis),
      rotationAngle(other.rotationAngle),
      model(other.model),
      name(std::move(other.name)),
      health(other.health),
      maxHealth(other.maxHealth),
      isInjured(other.isInjured),
      injuryType(std::move(other.injuryType)) {
  std::cout << "BodyPart move constructor called" << std::endl;
  other.health = 0.0f;
  other.maxHealth = 0.0f;
  other.isInjured = false;
  other.model = &GetSharedModel();
}

BodyPart& BodyPart::operator=(BodyPart&& other) noexcept {
  std::cout << "BodyPart move assignment operator called" << std::endl;
  if (this != &other) {
    size = other.size;
    color = other.color;
    rotationAxis = other.rotationAxis;
    rotationAngle = other.rotationAngle;
    name = std::move(other.name);
    health = other.health;
    maxHealth = other.maxHealth;
    isInjured = other.isInjured;
    injuryType = std::move(other.injuryType);
    model = other.model;

    other.health = 0.0f;
    other.maxHealth = 0.0f;
    other.isInjured = false;
    other.model = &GetSharedModel();
  }
  return *this;
}

BodyPart::~BodyPart() = default;

void swap(BodyPart& first, BodyPart& second) noexcept {
  using std::swap;
  swap(first.size, second.size);
  swap(first.color, second.color);
  swap(first.rotationAxis, second.rotationAxis);
  swap(first.rotationAngle, second.rotationAngle);
  swap(first.model, second.model);
  swap(first.name, second.name);
  swap(first.health, second.health);
  swap(first.maxHealth, second.maxHealth);
  swap(first.isInjured, second.isInjured);
  swap(first.injuryType, second.injuryType);
}

void BodyPart::setRotation(const Vector3 axis, float angle) {
  rotationAxis = axis;
  rotationAngle = angle;
}

void BodyPart::setPosition(Vector3 newPos) { GameObject::setPosition(newPos); }

void BodyPart::setColor(Color newColor) { color = newColor; }

Vector3 BodyPart::getSize() const { return size; }
Color BodyPart::getColor() const { return color; }

float BodyPart::getRotationAngle() const { return rotationAngle; }
Vector3 BodyPart::getRotationAxis() const { return rotationAxis; }

void BodyPart::draw() const {
  DrawModelEx(*model, position, rotationAxis, rotationAngle, size, color);
}

BoundingBox BodyPart::getBoundingBox() const {
  Vector3 currentPosition = getPosition();
  float halfX = size.x * 0.5f;
  float halfY = size.y * 0.5f;
  float halfZ = size.z * 0.5f;

  return (BoundingBox){
      (Vector3){currentPosition.x - halfX, currentPosition.y - halfY,
                currentPosition.z - halfZ},
      (Vector3){currentPosition.x + halfX, currentPosition.y + halfY,
                currentPosition.z + halfZ},
  };
}

std::unique_ptr<GameObject> BodyPart::clone() const {
  return std::make_unique<BodyPart>(*this);
}

void BodyPart::takeDamage(float damage) {
  health = std::max(0.0f, health - damage);
  if (health <= maxHealth * 0.3f) {
    isInjured = true;
    injuryType = "Severe damage";
  }
}

void BodyPart::heal(float amount) {
  health = std::min(maxHealth, health + amount);
  if (health > maxHealth * 0.7f) {
    isInjured = false;
    injuryType = "";
  }
}

float BodyPart::getHealthPercentage() const {
  return maxHealth > 0 ? (health / maxHealth) * 100.0f : 0.0f;
}

void BodyPart::displayStatus() const {
  std::cout << "BodyPart: " << name << " | Health: " << health << "/"
            << maxHealth << " (" << getHealthPercentage() << "%)"
            << " | Injured: " << (isInjured ? "Yes" : "No");
  if (isInjured && !injuryType.empty()) {
    std::cout << " (" << injuryType << ")";
  }
  std::cout << std::endl;
}