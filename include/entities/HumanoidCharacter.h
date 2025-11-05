// HumanoidCharacter.h - Solid humanoid character controller (no ragdoll)
#pragma once

#include <btBulletDynamicsCommon.h>

#include <memory>

#include "BodyPart.h"
#include "objects/GameObject.h"
#include "raylib.h"

class GameWorld;

// Character states for animation
enum HumanoidState { IDLE, WALKING, JUMPING };

// Visual part for rendering only (no individual physics)
struct HumanoidVisualPart {
  BodyPart visual;
  Vector3 baseOffset;     // Offset from character center
  Vector3 currentOffset;  // Current animated offset

  HumanoidVisualPart(Vector3 size, Color color, Vector3 offset)
      : visual({0, 0, 0}, size, color, true),
        baseOffset(offset),
        currentOffset(offset) {}
};

// Physics body part with individual collision
struct HumanoidPhysicsPart {
  btRigidBody* body = nullptr;
  btCollisionShape* shape = nullptr;
  btDefaultMotionState* motionState = nullptr;
  Vector3 baseOffset;    // Offset from character center
  Vector3 size;          // Dimensions for physics shape
  short collisionGroup;  // Collision group identifier

  HumanoidPhysicsPart(Vector3 partSize, Vector3 offset, short group)
      : baseOffset(offset), size(partSize), collisionGroup(group) {}

  ~HumanoidPhysicsPart() { cleanup(); }

  void cleanup() {
    delete body;
    delete shape;
    delete motionState;
    body = nullptr;
    shape = nullptr;
    motionState = nullptr;
  }
};

// Facial feature for eyes and expressions
struct FacialFeature {
  BodyPart visual;
  Vector3 baseOffset;
  Vector3 currentOffset;

  FacialFeature(Vector3 size, Color color, Vector3 offset)
      : visual({0, 0, 0}, size, color, true),
        baseOffset(offset),
        currentOffset(offset) {}
};

class HumanoidCharacter : public GameObject {
 public:
  static int totalCharactersCreated;
  static int activeCharacters;
  static constexpr float DEFAULT_MOVEMENT_SPEED = 5.0f;
  static constexpr float DEFAULT_JUMP_FORCE = 15.0f;

  static float calculateDistance(const HumanoidCharacter& char1,
                                 const HumanoidCharacter& char2);
  static Vector3 getCharacterSpacing(int characterCount);
  static bool areCharactersOverlapping(const HumanoidCharacter& char1,
                                       const HumanoidCharacter& char2);
  static int getTotalCharactersCreated() { return totalCharactersCreated; }
  static int getActiveCharacterCount() { return activeCharacters; }

  HumanoidCharacter(Vector3 position, GameWorld* gameWorld = nullptr);
  ~HumanoidCharacter();

  void setupPhysics(btDiscreteDynamicsWorld* bulletWorld);
  void removeFromPhysics(btDiscreteDynamicsWorld* bulletWorld);
  void update(float deltaTime) override;
  void draw() const override;
  BoundingBox getBoundingBox() const override;
  std::unique_ptr<GameObject> clone() const override;
  PhysicsBodyConfig getPhysicsConfig() const override;
  void configurePhysicsBody(btRigidBody& body) const override;
  void onRemovedFromWorld(GameWorld& world) override;

  std::string getObstacleType() const override { return "character"; }

  // Movement and control
  void jump();
  void applyMovement(const Vector3& direction, float speed,
                     float forceMultiplier = 1.0f);

  // Movement for AI (NPCs)
  void moveTowards(Vector3 target, float deltaTime);

  // Ground detection
  bool isOnGround() const;

  // Position methods
  Vector3 getFeetPosition() const;
  Vector3 getTorsoPosition() const;

  // Body part accessors for customization
  HumanoidVisualPart& getTorso() { return torso; }
  HumanoidVisualPart& getLeftArm() { return leftArm; }
  HumanoidVisualPart& getRightArm() { return rightArm; }
  HumanoidVisualPart& getLeftLeg() { return leftLeg; }
  HumanoidVisualPart& getRightLeg() { return rightLeg; }

  // Multi-body physics methods
  void createIndividualPhysicsBodies();
  void updatePhysicsBodyPositions();
  void synchronizeVisualWithPhysics();
  void removeAllPhysicsBodies();

  // Collision detection for individual body parts
  bool checkPartCollision(const HumanoidVisualPart& part,
                          const BoundingBox& obstacle) const;
  bool checkAnyPartCollision(const BoundingBox& obstacle) const;
  bool wouldCollideAfterMovement(Vector3 movement, float deltaTime) const;
  Vector3 getPartWorldPosition(const HumanoidVisualPart& part) const;
  BoundingBox getPartBoundingBox(const HumanoidVisualPart& part) const;

  void setWorld(GameWorld* w) { world = w; }

 private:
  void applyMovementInternal(Vector3 movement, float speed,
                             float forceMultiplier);
  void updateRotationTowardsMovement(Vector3 movement);

  // Main physics body for movement (torso-centered)
  btRigidBody* characterBody = nullptr;
  btCollisionShape* characterShape = nullptr;
  btDefaultMotionState* motionState = nullptr;

  // Individual physics bodies for collision detection
  HumanoidPhysicsPart headPhysics;
  HumanoidPhysicsPart torsoPhysics;
  HumanoidPhysicsPart leftArmPhysics;
  HumanoidPhysicsPart rightArmPhysics;
  HumanoidPhysicsPart leftLegPhysics;
  HumanoidPhysicsPart rightLegPhysics;

  // Physics world reference
  btDiscreteDynamicsWorld* physicsWorld = nullptr;

  // Body parts (visual only)
  HumanoidVisualPart head;
  HumanoidVisualPart torso;
  HumanoidVisualPart leftArm;
  HumanoidVisualPart rightArm;
  HumanoidVisualPart leftLeg;
  HumanoidVisualPart rightLeg;

  // Facial features (visual only)
  FacialFeature leftEye;
  FacialFeature rightEye;
  FacialFeature mouth;
  FacialFeature leftEyebrow;
  FacialFeature rightEyebrow;

  GameWorld* world = nullptr;

  // Character state and animation
  HumanoidState currentState = IDLE;
  float animationTime = 0.0f;
  bool isJumping = false;
  float jumpCooldown = 0.0f;

  Vector3 targetMovementDirection = {0, 0, 0};
  float currentMovementSpeed = 0.0f;

  void createSinglePhysicsBody();
  void createIndividualPhysicsBody(HumanoidPhysicsPart& part,
                                   Vector3 worldPosition);
  void updatePhysicsBodyPosition(HumanoidPhysicsPart& part,
                                 Vector3 worldPosition);
  void updateVisualPositions();
  void animateCharacter(float deltaTime);
  void updateCharacterState();
  void constrainBodyPartsToCharacter();
};
