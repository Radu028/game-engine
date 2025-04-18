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
    : GameObject(position, true) {
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

// BoundingBox CubeObject::getBoundingBox() const {
//   return (BoundingBox){
//       (Vector3){position.x - width / 2, position.y - height / 2,
//                 position.z - length / 2},
//       (Vector3){position.x + width / 2, position.y + height / 2,
//                 position.z + length / 2},
//   };
// }

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
  btTransform trans;
  rigidBody->getMotionState()->getWorldTransform(trans);
  btVector3 origin = trans.getOrigin();

  Vector3 currentPosition = {origin.getX(), origin.getY(), origin.getZ()};

  DrawCube(currentPosition,
           rigidBody->getCollisionShape()->getLocalScaling().getX() * 2,
           rigidBody->getCollisionShape()->getLocalScaling().getY() * 2,
           rigidBody->getCollisionShape()->getLocalScaling().getZ() * 2, color);
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