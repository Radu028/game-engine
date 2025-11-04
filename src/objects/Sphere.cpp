#include "objects/Sphere.h"

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
