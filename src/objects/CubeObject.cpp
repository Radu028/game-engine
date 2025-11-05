#include "objects/CubeObject.h"

#include <btBulletDynamicsCommon.h>

#include <memory>
#include <string>

#include "physics/PhysicsTypes.h"
#include "systems/ShaderSystem.h"

CubeObject::CubeObject(Vector3 position, Vector3 size, Color color,
                       bool hasCollision, const std::string& texturePath,
                       bool affectedByGravity, bool isStatic)
    : GameObject(position, hasCollision, affectedByGravity, isStatic),
      size(size),
      color(color),
      hasTexture(false),
      textureLoaded(false),
      model(LoadModelFromMesh(GenMeshCube(size.x, size.y, size.z))) {
  ShaderSystem* shaderSystem = ShaderSystem::getInstance();
  model.materials[0].shader = shaderSystem->getShader();

  if (!texturePath.empty()) {
    texture = LoadTexture(texturePath.c_str());
    if (texture.id > 0) {
      textureLoaded = true;
      hasTexture = true;
      model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = texture;
    } else {
      TraceLog(LOG_WARNING, "Failed to load texture: %s", texturePath.c_str());
    }
  }
}

CubeObject::CubeObject(const CubeObject& other)
    : GameObject(other.position, other.hasCollision, other.affectedByGravity,
                 other.isStatic),
      size(other.size),
      color(other.color),
      hasTexture(other.hasTexture),
      textureLoaded(other.textureLoaded),
      model(LoadModelFromMesh(GenMeshCube(size.x, size.y, size.z))) {
  ShaderSystem* shaderSystem = ShaderSystem::getInstance();
  model.materials[0].shader = shaderSystem->getShader();

  if (hasTexture) {
    texture = other.texture;
    model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = texture;
  }
}

CubeObject::~CubeObject() {
  UnloadModel(model);

  if (hasTexture) {
    UnloadTexture(texture);
  }
}

Vector3 CubeObject::getSize() const { return size; }

BoundingBox CubeObject::getBoundingBox() const {
  float halfX = size.x * 0.5f;
  float halfY = size.y * 0.5f;
  float halfZ = size.z * 0.5f;

  return (BoundingBox){
      (Vector3){position.x - halfX, position.y - halfY, position.z - halfZ},
      (Vector3){position.x + halfX, position.y + halfY, position.z + halfZ},
  };
}

void CubeObject::draw() const {
  Color tint = hasTexture ? WHITE : color;
  DrawModel(model, position, 1.0f, tint);

#ifdef DEBUG_WIREFRAMES
  Color wireColor = RED;
  if (color.r == 255) wireColor = GREEN;

  BoundingBox box = getBoundingBox();
  DrawBoundingBox(box, wireColor);
#endif
}

bool CubeObject::checkCollision(const CubeObject& other) const {
  if (!hasCollision || !other.getHasCollision()) {
    return false;
  }

  BoundingBox box1 = this->getBoundingBox();
  BoundingBox box2 = other.getBoundingBox();

  bool result = CheckCollisionBoxes(box1, box2);
  return result;
}

void CubeObject::interact() {}

std::unique_ptr<GameObject> CubeObject::clone() const {
  return std::make_unique<CubeObject>(*this);
}

PhysicsBodyConfig CubeObject::getPhysicsConfig() const {
  PhysicsBodyConfig config;
  config.usesPhysics = hasCollision;
  config.collider = ColliderType::Box;
  config.dimensions = size;
  config.isStatic = isStatic;
  config.affectedByGravity = affectedByGravity;
  return config;
}

void CubeObject::configurePhysicsBody(btRigidBody& body) const {
  body.setActivationState(DISABLE_DEACTIVATION);

  if (isStatic) {
    body.setFriction(1.5f);
    body.setRollingFriction(1.5f);
    body.setSpinningFriction(1.5f);
    body.setDamping(0.0f, 0.0f);
    body.setCollisionFlags(body.getCollisionFlags() |
                           btCollisionObject::CF_STATIC_OBJECT);
    return;
  }

  body.setFriction(2.0f);
  body.setRollingFriction(2.0f);
  body.setSpinningFriction(2.0f);
  body.setDamping(0.8f, 0.8f);
}