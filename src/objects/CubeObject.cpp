#include "objects/CubeObject.h"

#include <memory>
#include <string>

#include "systems/ShaderSystem.h"

CubeObject::CubeObject(Vector3 position, Vector3 size, Color color,
                       bool hasCollision, const std::string& texturePath,
                       bool affectedByGravity, bool isStatic, bool useShaders)
    : GameObject(position, hasCollision, affectedByGravity, isStatic),
      size(size),
      color(color),
      textureLoaded(false),
      useShaders(useShaders) {
  model = LoadModelFromMesh(GenMeshCube(size.x, size.y, size.z));

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

  // If using shaders, assign the lighting shader to the model material
  if (useShaders) {
    ShaderSystem* shaderSystem = ShaderSystem::getInstance();
    model.materials[0].shader = shaderSystem->getShader();
  }
}

CubeObject::CubeObject(const CubeObject& other)
    : GameObject(other.position, other.hasCollision, other.affectedByGravity,
                 other.isStatic),
      size(other.size),
      color(other.color),
      hasTexture(other.hasTexture),
      useShaders(other.useShaders) {
  model = LoadModelFromMesh(GenMeshCube(size.x, size.y, size.z));

  if (hasTexture) {
    texture = other.texture;
    model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = texture;
  }

  // If using shaders, assign the lighting shader to the model material
  if (useShaders) {
    ShaderSystem* shaderSystem = ShaderSystem::getInstance();
    model.materials[0].shader = shaderSystem->getShader();
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
  if (useShaders && hasTexture) {
    // With shaders and texture, the material already has the shader assigned
    DrawModel(model, position, 1.0f, WHITE);
  } else if (useShaders) {
    // With shaders but no texture, use the model with color
    DrawModel(model, position, 1.0f, color);
  } else if (hasTexture) {
    // Fallback to normal rendering with texture
    DrawModel(model, position, 1.0f, WHITE);
  } else {
    // Fallback to simple cube rendering
    DrawCube(position, size.x, size.y, size.z, color);
  }

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