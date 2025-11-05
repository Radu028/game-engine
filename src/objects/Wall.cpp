#include "objects/Wall.h"

#include <btBulletDynamicsCommon.h>
#include "physics/PhysicsTypes.h"
#include "systems/ShaderSystem.h"

Wall::Wall(Vector3 position, Vector3 dimensions, Color color, bool hasCollision,
           bool useShaders)
    : StaticWorldObject(position, hasCollision),
      dimensions(dimensions),
      color(color),
      useShaders(useShaders) {
  model =
      LoadModelFromMesh(GenMeshCube(dimensions.x, dimensions.y, dimensions.z));

  ShaderSystem* shaderSystem = ShaderSystem::getInstance();
  model.materials[0].shader = shaderSystem->getShader();
}

void Wall::draw() const {
  if (useShaders) {
    DrawModel(model, position, 1.0f, color);
  } else {
    DrawCube(position, dimensions.x, dimensions.y, dimensions.z, color);
  }
}

BoundingBox Wall::getBoundingBox() const {
  float halfX = dimensions.x * 0.5f;
  float halfY = dimensions.y * 0.5f;
  float halfZ = dimensions.z * 0.5f;

  return (BoundingBox){
      (Vector3){position.x - halfX, position.y - halfY, position.z - halfZ},
      (Vector3){position.x + halfX, position.y + halfY, position.z + halfZ},
  };
}

std::unique_ptr<GameObject> Wall::clone() const {
  return std::make_unique<Wall>(*this);
}

PhysicsBodyConfig Wall::getPhysicsConfig() const {
  PhysicsBodyConfig config;
  config.usesPhysics = hasCollision;
  config.collider = ColliderType::Box;
  config.dimensions = dimensions;
  config.isStatic = true;
  config.affectedByGravity = false;
  return config;
}

void Wall::configurePhysicsBody(btRigidBody& body) const {
  body.setFriction(1.0f);
  body.setRollingFriction(0.4f);
  body.setSpinningFriction(0.4f);
  body.setDamping(0.2f, 0.2f);
  body.setCollisionFlags(body.getCollisionFlags() |
                         btCollisionObject::CF_STATIC_OBJECT);
}