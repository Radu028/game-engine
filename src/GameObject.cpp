#include "GameObject.h"

#include <cmath>

// GameObject::GameObject(Vector3 position, Vector3 size, Color color, bool
// hasCollision)
//     : position(position), size(size), color(color),
//     hasCollision(hasCollision) {}

GameObject::GameObject(Vector3 position, bool hasCollision)
    : position(position), hasCollision(hasCollision) {}

// BoundingBox GameObject::getBoundingBox() const {
//     return (BoundingBox){
//         (Vector3){position.x - size.x / 2, position.y - size.y / 2,
//         position.z - size.z / 2}, (Vector3){position.x + size.x / 2,
//         position.y + size.y / 2, position.z + size.z / 2},
//     };
// }

// Provide a default (no-op) implementation for the pure virtual draw method
void GameObject::draw() const {
  // Base class draw operation is a no-op; derived classes should override this
}

// bool GameObject::checkCollision(const GameObject& other) const {
//     if (!this->hasCollision || !other.getHasCollision()) {
//         return false;
//     }

//     BoundingBox box1 = this->getBoundingBox();
//     BoundingBox box2 = other.getBoundingBox();

//     bool result = CheckCollisionBoxes(box1, box2);
//     return result;
// }

float GameObject::getDistance(const GameObject& other) const {
  return sqrtf((other.position.x - this->position.x) *
                   (other.position.x - this->position.x) +
               (other.position.y - this->position.y) *
                   (other.position.y - this->position.y) +
               (other.position.z - this->position.z) *
                   (other.position.z - this->position.z));
}

// void GameObject::handleCollision(GameObject& other) {
//     // For derivated classes to overwrite this.
// }