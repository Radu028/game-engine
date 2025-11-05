#include "objects/Sphere.h"

#include <btBulletDynamicsCommon.h>
#include "physics/PhysicsTypes.h"
#include "systems/ShaderSystem.h"

Sphere::Sphere(Vector3 position, float radius, Color color, bool hasCollisions,
               bool useShaders)
    : StaticWorldObject(position, hasCollisions),
      radius(radius),
      color(color),
      useShaders(useShaders) {
  model = LoadModelFromMesh(GenMeshSphere(radius, 32, 32));

  ShaderSystem* shaderSystem = ShaderSystem::getInstance();
  model.materials[0].shader = shaderSystem->getShader();
}

void Sphere::draw() const {
  if (useShaders) {
    DrawModel(model, position, 1.0f, color);
  } else {
    DrawSphere(position, radius, color);
  }
}

BoundingBox Sphere::getBoundingBox() const {
  float half = radius;
  return (BoundingBox){
      (Vector3){position.x - half, position.y - half, position.z - half},
      (Vector3){position.x + half, position.y + half, position.z + half},
  };
}

std::unique_ptr<GameObject> Sphere::clone() const {
  return std::make_unique<Sphere>(*this);
}

PhysicsBodyConfig Sphere::getPhysicsConfig() const {
  PhysicsBodyConfig config;
  config.usesPhysics = hasCollision;
  config.collider = ColliderType::Sphere;
  config.radius = radius;
  config.isStatic = true;
  config.affectedByGravity = false;
  return config;
}

void Sphere::configurePhysicsBody(btRigidBody& body) const {
  body.setFriction(1.2f);
  body.setRollingFriction(0.5f);
  body.setSpinningFriction(0.5f);
  body.setDamping(0.1f, 0.1f);
  body.setCollisionFlags(body.getCollisionFlags() |
                         btCollisionObject::CF_STATIC_OBJECT);
}
