#include "objects/Floor.h"

#include <btBulletDynamicsCommon.h>

#include <string>

#include "physics/PhysicsTypes.h"
#include "systems/ShaderSystem.h"

Floor::Floor(Vector3 position, Vector3 dimensions, Color color,
             bool hasCollision, bool useShaders)
    : StaticWorldObject(position, hasCollision),
      dimensions(dimensions),
      color(color),
      hasTexture(false),
      useShaders(useShaders) {
  if (useShaders) {
    model = LoadModelFromMesh(
        GenMeshCube(dimensions.x, dimensions.y, dimensions.z));

    ShaderSystem* shaderSystem = ShaderSystem::getInstance();
    model.materials[0].shader = shaderSystem->getShader();
  }
}

Floor::Floor(Vector3 position, Vector3 dimensions, std::string texturePath,
             bool hasCollision)
    : StaticWorldObject(position, true), dimensions(dimensions), color(WHITE) {
  model =
      LoadModelFromMesh(GenMeshCube(dimensions.x, dimensions.y, dimensions.z));

  Texture2D texture = LoadTexture(texturePath.c_str());
  if (texture.id > 0) {
    hasTexture = true;
    model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = texture;
  } else {
    TraceLog(LOG_WARNING, "Failed to load texture: %s", texturePath.c_str());
  }
}

void Floor::draw() const {
  if (hasTexture) {
    DrawModel(model, position, 1.0f, WHITE);
  } else if (useShaders) {
    DrawModel(model, position, 1.0f, color);
  } else {
    DrawCube(position, dimensions.x, dimensions.y, dimensions.z, color);
  }
}

BoundingBox Floor::getBoundingBox() const {
  float halfX = dimensions.x * 0.5f;
  float halfY = dimensions.y * 0.5f;
  float halfZ = dimensions.z * 0.5f;

  return (BoundingBox){
      (Vector3){position.x - halfX, position.y - halfY, position.z - halfZ},
      (Vector3){position.x + halfX, position.y + halfY, position.z + halfZ},
  };
}

std::unique_ptr<GameObject> Floor::clone() const {
  return std::make_unique<Floor>(*this);
}

PhysicsBodyConfig Floor::getPhysicsConfig() const {
  PhysicsBodyConfig config;
  config.usesPhysics = hasCollision;
  config.collider = ColliderType::Box;
  config.dimensions = dimensions;
  config.isStatic = true;
  config.affectedByGravity = false;
  return config;
}

void Floor::configurePhysicsBody(btRigidBody& body) const {
  body.setFriction(1.5f);
  body.setRollingFriction(1.5f);
  body.setSpinningFriction(1.5f);
  body.setDamping(0.0f, 0.0f);
  body.setCollisionFlags(body.getCollisionFlags() |
                         btCollisionObject::CF_STATIC_OBJECT);
}