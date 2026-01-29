#include "entities/BodyPart.h"

#include "systems/ShaderSystem.h"

namespace {
struct SharedCubeModel {
  Model model{};

  SharedCubeModel() {
    model = LoadModelFromMesh(GenMeshCube(1.0f, 1.0f, 1.0f));

    // Use the engine shader if available (application initializes ShaderSystem
    // before creating characters).
    if (ShaderSystem* shaderSystem = ShaderSystem::getInstance()) {
      model.materials[0].shader = shaderSystem->getShader();
    }
  }

  ~SharedCubeModel() { UnloadModel(model); }
};

Model& getSharedCubeModel() {
  static SharedCubeModel shared;
  return shared.model;
}
}  // namespace

BodyPart::BodyPart(Vector3 position, Vector3 size, Color color,
                   bool hasCollision)
    : GameObject(position, hasCollision, /*affectedByGravity=*/false,
                 /*isStatic=*/true),
      size_(size),
      color_(color),
      model_(&getSharedCubeModel()) {}

void BodyPart::setRotation(Vector3 axis, float angleDegrees) {
  rotationAxis_ = axis;
  rotationAngleDegrees_ = angleDegrees;
}

void BodyPart::draw() const {
  if (!model_) return;
  DrawModelEx(*model_, position, rotationAxis_, rotationAngleDegrees_, size_,
              color_);
}

BoundingBox BodyPart::getBoundingBox() const {
  const float halfX = size_.x * 0.5f;
  const float halfY = size_.y * 0.5f;
  const float halfZ = size_.z * 0.5f;

  return (BoundingBox){
      (Vector3){position.x - halfX, position.y - halfY, position.z - halfZ},
      (Vector3){position.x + halfX, position.y + halfY, position.z + halfZ},
  };
}

std::unique_ptr<GameObject> BodyPart::clone() const {
  auto part = std::make_unique<BodyPart>(position, size_, color_, hasCollision);
  part->setRotation(rotationAxis_, rotationAngleDegrees_);
  return part;
}


