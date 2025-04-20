#include "CubeObject.h"

#include "PhysicsEngine.h"
#include "raylib.h"

// CubeObject::CubeObject(Vector3 position, float width, float height,
//                        float length, Color color, bool hasCollision,
//                        const char* texturePath)
//     : GameObject(position, hasCollision),
//       width(width),
//       height(height),
//       length(length),
//       color(color) {
//   model = LoadModelFromMesh(GenMeshCube(width, height, length));

//   if (texturePath != nullptr) {
//     texture = LoadTexture(texturePath);
//     model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = texture;
//     hasTexture = true;
//   } else {
//     hasTexture = false;
//   }
// }

CubeObject::CubeObject(const Vector3& position, float width, float height,
                       float depth, Color color, bool isStatic,
                       const std::string& texturePath)
    : GameObject(position, true),
      width(width),
      height(height),
      depth(depth),
      color(color) {
  collisionShape =
      new btBoxShape(btVector3(width * 0.5f, height * 0.5f, depth * 0.5f));

  btTransform startTransform;
  startTransform.setIdentity();
  startTransform.setOrigin(btVector3(position.x, position.y, position.z));

  float mass = isStatic ? 0.0f : 1.0f;
  btVector3 localInertia(0, 0, 0);
  if (mass > 0.0f) {
    collisionShape->calculateLocalInertia(mass, localInertia);
  }

  motionState = new btDefaultMotionState(startTransform);

  btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motionState,
                                                  collisionShape, localInertia);
  rigidBody = new btRigidBody(rbInfo);

  PhysicsEngine::getInstance().getDynamicsWorld()->addRigidBody(rigidBody);
}

// CubeObject::~CubeObject() {
//   UnloadModel(model);

//   if (hasTexture) {
//     UnloadTexture(texture);
//   }
// }

CubeObject::~CubeObject() {
  PhysicsEngine::getInstance().getDynamicsWorld()->removeRigidBody(rigidBody);
  delete rigidBody;
  delete motionState;
  delete collisionShape;
}

// Calculate the bounding box of the cube based on its collision shape and
// transform
BoundingBox CubeObject::getBoundingBox() const {
  // Get world transform of the rigid body
  btTransform trans;
  motionState->getWorldTransform(trans);
  btVector3 origin = trans.getOrigin();
  // Convert to raylib Vector3
  Vector3 pos = {origin.getX(), origin.getY(), origin.getZ()};
  // Get half extents of the box shape (with margin)
  const btBoxShape* boxShape = static_cast<const btBoxShape*>(collisionShape);
  btVector3 halfExtents = boxShape->getHalfExtentsWithMargin();
  // Construct and return the bounding box
  return BoundingBox{{pos.x - halfExtents.x(), pos.y - halfExtents.y(),
                      pos.z - halfExtents.z()},
                     {pos.x + halfExtents.x(), pos.y + halfExtents.y(),
                      pos.z + halfExtents.z()}};
}

// void CubeObject::draw() const {
//   if (hasTexture) {
//     DrawModel(model, position, 1.0f, WHITE);
//   } else {
//     DrawCube(position, width, height, length, color);
//   }

//   Color wireColor = RED;
//   if (color.r == 255) wireColor = GREEN;

//   BoundingBox box = getBoundingBox();
//   DrawBoundingBox(box, wireColor);
// }

void CubeObject::draw() const {
  // Draw cube at its stored position and dimensions
  DrawCube(position, width, height, depth, color);
}

// Synchronize position from physics before drawing
void CubeObject::update(float deltaTime) {
  btTransform trans;
  motionState->getWorldTransform(trans);
  btVector3 origin = trans.getOrigin();
  // Update the base class position
  position = {origin.getX(), origin.getY(), origin.getZ()};
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