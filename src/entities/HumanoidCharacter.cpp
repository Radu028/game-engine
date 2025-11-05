// HumanoidCharacter.cpp - Solid humanoid character controller (no ragdoll)
#include "entities/HumanoidCharacter.h"

#include <cmath>

#include "GameWorld.h"
#include "raylib.h"
#include "raymath.h"
#include "settings/Physics.h"

int HumanoidCharacter::totalCharactersCreated = 0;
int HumanoidCharacter::activeCharacters = 0;

float HumanoidCharacter::calculateDistance(const HumanoidCharacter& char1,
                                           const HumanoidCharacter& char2) {
  Vector3 pos1 = char1.getPosition();
  Vector3 pos2 = char2.getPosition();
  return Vector3Distance(pos1, pos2);
}

Vector3 HumanoidCharacter::getCharacterSpacing(int characterCount) {
  if (characterCount <= 1) return {0.0f, 0.0f, 0.0f};
  float spacing =
      3.0f + (characterCount * 0.5f);  // Increase spacing with more characters
  return {spacing, 0.0f, spacing};
}

bool HumanoidCharacter::areCharactersOverlapping(
    const HumanoidCharacter& char1, const HumanoidCharacter& char2) {
  return calculateDistance(char1, char2) < 2.0f;  // Less than 2 units apart
}

HumanoidCharacter::HumanoidCharacter(Vector3 position, GameWorld* gameWorld)
    : GameObject(position, true, true, false),
      // Initialize body parts with calculated offsets
      head(GameSettings::BodyParts::HEAD_SIZE,
           GameSettings::BodyParts::HEAD_COLOR,
           {0, GameSettings::CharacterCalculations::getHeadBaseYOffset(), 0}),
      torso(GameSettings::BodyParts::TORSO_SIZE,
            GameSettings::BodyParts::TORSO_COLOR,
            {0, GameSettings::CharacterCalculations::getTorsoBaseYOffset(), 0}),
      leftArm(GameSettings::BodyParts::ARM_SIZE,
              GameSettings::BodyParts::ARM_COLOR,
              {-GameSettings::BodyParts::ARM_OFFSET_X,
               GameSettings::CharacterCalculations::getArmBaseYOffset(), 0}),
      rightArm(GameSettings::BodyParts::ARM_SIZE,
               GameSettings::BodyParts::ARM_COLOR,
               {GameSettings::BodyParts::ARM_OFFSET_X,
                GameSettings::CharacterCalculations::getArmBaseYOffset(), 0}),
      leftLeg(GameSettings::BodyParts::LEG_SIZE,
              GameSettings::BodyParts::LEG_COLOR,
              {-GameSettings::BodyParts::LEG_OFFSET_X,
               GameSettings::CharacterCalculations::getLegBaseYOffset(), 0}),
      rightLeg(GameSettings::BodyParts::LEG_SIZE,
               GameSettings::BodyParts::LEG_COLOR,
               {GameSettings::BodyParts::LEG_OFFSET_X,
                GameSettings::CharacterCalculations::getLegBaseYOffset(), 0}),
      // Initialize physics components for body parts
      headPhysics(
          GameSettings::BodyParts::HEAD_SIZE,
          {0, GameSettings::CharacterCalculations::getHeadBaseYOffset(), 0},
          GameSettings::Collision::Groups::CHARACTER_HEAD),
      torsoPhysics(
          GameSettings::BodyParts::TORSO_SIZE,
          {0, GameSettings::CharacterCalculations::getTorsoBaseYOffset(), 0},
          GameSettings::Collision::Groups::CHARACTER_TORSO),
      leftArmPhysics(
          GameSettings::BodyParts::ARM_SIZE,
          {-GameSettings::BodyParts::ARM_OFFSET_X,
           GameSettings::CharacterCalculations::getArmBaseYOffset(), 0},
          GameSettings::Collision::Groups::CHARACTER_ARMS),
      rightArmPhysics(
          GameSettings::BodyParts::ARM_SIZE,
          {GameSettings::BodyParts::ARM_OFFSET_X,
           GameSettings::CharacterCalculations::getArmBaseYOffset(), 0},
          GameSettings::Collision::Groups::CHARACTER_ARMS),
      leftLegPhysics(
          GameSettings::BodyParts::LEG_SIZE,
          {-GameSettings::BodyParts::LEG_OFFSET_X,
           GameSettings::CharacterCalculations::getLegBaseYOffset(), 0},
          GameSettings::Collision::Groups::CHARACTER_LEGS),
      rightLegPhysics(
          GameSettings::BodyParts::LEG_SIZE,
          {GameSettings::BodyParts::LEG_OFFSET_X,
           GameSettings::CharacterCalculations::getLegBaseYOffset(), 0},
          GameSettings::Collision::Groups::CHARACTER_LEGS),
      // Eye features positioned on front of head
      leftEye(
          {0.08f, 0.08f, 0.12f}, BLACK,
          {-0.12f,
           GameSettings::CharacterCalculations::getHeadBaseYOffset() + 0.08f,
           0.25f}),
      rightEye(
          {0.08f, 0.08f, 0.12f}, BLACK,
          {0.12f,
           GameSettings::CharacterCalculations::getHeadBaseYOffset() + 0.08f,
           0.25f}),
      mouth(
          {0.15f, 0.04f, 0.08f}, MAROON,
          {0, GameSettings::CharacterCalculations::getHeadBaseYOffset() - 0.08f,
           0.25f}),
      // Facial features for character expression
      leftEyebrow(
          {0.10f, 0.02f, 0.06f}, DARKBROWN,
          {-0.12f,
           GameSettings::CharacterCalculations::getHeadBaseYOffset() + 0.15f,
           0.26f}),
      rightEyebrow(
          {0.10f, 0.02f, 0.06f}, DARKBROWN,
          {0.12f,
           GameSettings::CharacterCalculations::getHeadBaseYOffset() + 0.15f,
           0.26f}) {
  totalCharactersCreated++;
  activeCharacters++;

  if (gameWorld) {
    setWorld(gameWorld);
    setupPhysics(gameWorld->getDynamicsWorld());
  }
}

HumanoidCharacter::~HumanoidCharacter() {
  activeCharacters--;

  if (physicsWorld && characterBody) {
    physicsWorld->removeRigidBody(characterBody);
  }

  // Remove all individual physics bodies
  removeAllPhysicsBodies();

  delete characterBody;
  delete characterShape;
  delete motionState;
}

void HumanoidCharacter::createSinglePhysicsBody() {
  // Use dimensions from settings
  float radius = GameSettings::Character::RADIUS;
  float height = GameSettings::Character::HEIGHT;

  characterShape = new btCapsuleShape(radius, height);
  characterShape->setMargin(GameSettings::Collision::SHAPE_MARGIN);

  // Position character body so feet touch the ground
  // position.y represents foot level, physics center is offset accordingly
  btTransform transform;
  transform.setIdentity();

  // Ensure the physics body is properly positioned above ground
  float physicsCenterY =
      GameSettings::CharacterCalculations::getPhysicsCenterYFromGroundY(
          position.y);

  transform.setOrigin(btVector3(position.x, physicsCenterY, position.z));

  motionState = new btDefaultMotionState(transform);

  float mass = GameSettings::Character::MASS;
  btVector3 inertia(0, 0, 0);
  characterShape->calculateLocalInertia(mass, inertia);

  // Create rigid body with physics settings
  btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motionState,
                                                  characterShape, inertia);
  rbInfo.m_friction = GameSettings::Collision::FRICTION;
  rbInfo.m_rollingFriction = GameSettings::Collision::ROLLING_FRICTION;
  rbInfo.m_restitution = GameSettings::Collision::RESTITUTION;

  characterBody = new btRigidBody(rbInfo);

  // Use settings for damping
  characterBody->setDamping(GameSettings::Character::DAMPING_LINEAR,
                            GameSettings::Character::DAMPING_ANGULAR);
  characterBody->setActivationState(DISABLE_DEACTIVATION);
  characterBody->setSleepingThresholds(0.0f, 0.0f);

  // Lock rotation around X and Z axes to prevent tipping
  characterBody->setAngularFactor(btVector3(0, 1, 0));

  // Set gravity
  characterBody->setGravity(
      btVector3(0, GameSettings::Physics::GRAVITY_ACCELERATION, 0));
}

void HumanoidCharacter::setupPhysics(btDiscreteDynamicsWorld* bulletWorld) {
  physicsWorld = bulletWorld;
  createSinglePhysicsBody();

  // Add the main character body to the world (torso-based movement)
  // Main character body should collide with world objects
  short characterGroup = GameSettings::Collision::Groups::CHARACTER_TORSO;
  short characterMask =
      GameSettings::Collision::Groups::WORLD_OBJECTS;  // Fix: collide with
                                                       // world objects
  physicsWorld->addRigidBody(characterBody, characterGroup, characterMask);

  // Create individual physics bodies for each body part
  createIndividualPhysicsBodies();
}

void HumanoidCharacter::removeFromPhysics(
    btDiscreteDynamicsWorld* bulletWorld) {
  if (bulletWorld && characterBody) {
    bulletWorld->removeRigidBody(characterBody);
  }

  // Remove all individual physics bodies
  removeAllPhysicsBodies();

  physicsWorld = nullptr;
}

void HumanoidCharacter::update(float deltaTime) {
  if (!characterBody) return;

  if (jumpCooldown > 0.0f) {
    jumpCooldown -= deltaTime;
  }

  updateCharacterState();

  animateCharacter(deltaTime);

  // Update visual positions and facial features (combined for efficiency)
  updateVisualPositions();

  // Synchronize physics bodies with visual positions for collision detection
  synchronizeVisualWithPhysics();
}

void HumanoidCharacter::updateCharacterState() {
  Vector3 velocity = {0, 0, 0};
  if (characterBody) {
    btVector3 btVel = characterBody->getLinearVelocity();
    velocity = {btVel.getX(), btVel.getY(), btVel.getZ()};
  }

  // Reset jumping state when definitely landed - balanced conditions
  if (isJumping && isOnGround() && abs(velocity.y) < 0.5f) {
    isJumping = false;
  }

  float horizontalSpeed =
      sqrt(velocity.x * velocity.x + velocity.z * velocity.z);

  if (isJumping || !isOnGround() ||
      abs(velocity.y) > 1.0f) {  // Balanced jumping detection
    currentState = JUMPING;
  } else if (horizontalSpeed > 0.15f) {
    currentState = WALKING;
  } else {
    currentState = IDLE;
  }
}

void HumanoidCharacter::animateCharacter(float deltaTime) {
  animationTime += deltaTime;

  switch (currentState) {
    case IDLE:
      // Subtle breathing animation
      head.currentOffset = Vector3Add(
          head.baseOffset, {0, sin(animationTime * 2.0f) * 0.01f, 0});
      torso.currentOffset = torso.baseOffset;
      leftArm.currentOffset = leftArm.baseOffset;
      rightArm.currentOffset = rightArm.baseOffset;
      leftLeg.currentOffset = leftLeg.baseOffset;
      rightLeg.currentOffset = rightLeg.baseOffset;
      break;

    case WALKING: {
      // Walking animation with limb movement
      float walkCycle = animationTime * 8.0f;   // Walking cycle frequency
      float armSwingL = sin(walkCycle) * 0.4f;  // Left arm swing
      float armSwingR =
          sin(walkCycle + M_PI) * 0.4f;         // Right arm swing (opposite)
      float legSwingL = sin(walkCycle) * 0.3f;  // Left leg swing
      float legSwingR =
          sin(walkCycle + M_PI) * 0.3f;  // Right leg swing (opposite)

      // Head bobbing
      head.currentOffset =
          Vector3Add(head.baseOffset, {0, sin(walkCycle * 2.0f) * 0.05f, 0});

      // Torso movement for walking
      torso.currentOffset = Vector3Add(
          torso.baseOffset,
          {sin(walkCycle) * 0.02f, sin(walkCycle * 2.0f) * 0.03f, 0});

      // Arms swing naturally
      leftArm.currentOffset = Vector3Add(
          leftArm.baseOffset, {0, armSwingL * 0.3f, armSwingL * 0.6f});
      rightArm.currentOffset = Vector3Add(
          rightArm.baseOffset, {0, armSwingR * 0.3f, armSwingR * 0.6f});

      // Legs with natural walking motion - lifting and moving forward/back
      float leftLegLift =
          fmax(0, sin(walkCycle)) * 0.15f;  // Only lift when moving forward
      float rightLegLift = fmax(0, sin(walkCycle + M_PI)) *
                           0.15f;  // Only lift when moving forward

      leftLeg.currentOffset =
          Vector3Add(leftLeg.baseOffset,
                     {legSwingL * 0.1f, leftLegLift, legSwingL * 0.2f});
      rightLeg.currentOffset =
          Vector3Add(rightLeg.baseOffset,
                     {legSwingR * 0.1f, rightLegLift, legSwingR * 0.2f});
      break;
    }

    case JUMPING:
      // Jump pose - arms up slightly, legs together
      head.currentOffset = head.baseOffset;
      torso.currentOffset = torso.baseOffset;
      leftArm.currentOffset = Vector3Add(leftArm.baseOffset, {0, 0.1f, 0});
      rightArm.currentOffset = Vector3Add(rightArm.baseOffset, {0, 0.1f, 0});
      leftLeg.currentOffset = Vector3Add(leftLeg.baseOffset, {0.05f, 0, 0});
      rightLeg.currentOffset = Vector3Add(rightLeg.baseOffset, {-0.05f, 0, 0});
      break;
  }
}

void HumanoidCharacter::updateVisualPositions() {
  if (!characterBody) return;

  // Get physics body transform ONCE per frame - SINGLE SOURCE OF TRUTH
  btTransform transform;
  characterBody->getMotionState()->getWorldTransform(transform);
  btVector3 origin = transform.getOrigin();
  Vector3 physicsPos = {origin.getX(), origin.getY(), origin.getZ()};

  // Update main position once per frame (at feet level)
  // Calculate ground position from physics body center
  position = {origin.getX(),
              GameSettings::CharacterCalculations::getGroundYFromPhysicsCenterY(
                  origin.getY()),
              origin.getZ()};

  // Get rotation (only Y-axis rotation allowed)
  btQuaternion rotation = transform.getRotation();
  float yRotation = atan2(2.0f * (rotation.getW() * rotation.getY() +
                                  rotation.getX() * rotation.getZ()),
                          1.0f - 2.0f * (rotation.getY() * rotation.getY() +
                                         rotation.getZ() * rotation.getZ()));

  // Convert rotation to degrees for BodyPart's setRotation method
  float yRotationDegrees = yRotation * RAD2DEG;

  // Update visual parts positions and rotations
  Vector3 rotatedOffset;

  // Head position and rotation
  rotatedOffset =
      Vector3RotateByAxisAngle(head.currentOffset, {0, 1, 0}, yRotation);
  head.visual.setPosition(Vector3Add(physicsPos, rotatedOffset));
  head.visual.setRotation({0, 1, 0}, yRotationDegrees);

  // Torso position and rotation
  rotatedOffset =
      Vector3RotateByAxisAngle(torso.currentOffset, {0, 1, 0}, yRotation);
  torso.visual.setPosition(Vector3Add(physicsPos, rotatedOffset));
  torso.visual.setRotation({0, 1, 0}, yRotationDegrees);

  // Left Arm position and rotation
  rotatedOffset =
      Vector3RotateByAxisAngle(leftArm.currentOffset, {0, 1, 0}, yRotation);
  leftArm.visual.setPosition(Vector3Add(physicsPos, rotatedOffset));
  leftArm.visual.setRotation({0, 1, 0}, yRotationDegrees);

  // Right Arm position and rotation
  rotatedOffset =
      Vector3RotateByAxisAngle(rightArm.currentOffset, {0, 1, 0}, yRotation);
  rightArm.visual.setPosition(Vector3Add(physicsPos, rotatedOffset));
  rightArm.visual.setRotation({0, 1, 0}, yRotationDegrees);

  // Left Leg position and rotation
  rotatedOffset =
      Vector3RotateByAxisAngle(leftLeg.currentOffset, {0, 1, 0}, yRotation);
  leftLeg.visual.setPosition(Vector3Add(physicsPos, rotatedOffset));
  leftLeg.visual.setRotation({0, 1, 0}, yRotationDegrees);

  // Right Leg position and rotation
  rotatedOffset =
      Vector3RotateByAxisAngle(rightLeg.currentOffset, {0, 1, 0}, yRotation);
  rightLeg.visual.setPosition(Vector3Add(physicsPos, rotatedOffset));
  rightLeg.visual.setRotation({0, 1, 0}, yRotationDegrees);

  // Facial features transform
  rotatedOffset =
      Vector3RotateByAxisAngle(leftEye.currentOffset, {0, 1, 0}, yRotation);
  leftEye.visual.setPosition(Vector3Add(physicsPos, rotatedOffset));
  leftEye.visual.setRotation({0, 1, 0}, yRotationDegrees);

  rotatedOffset =
      Vector3RotateByAxisAngle(rightEye.currentOffset, {0, 1, 0}, yRotation);
  rightEye.visual.setPosition(Vector3Add(physicsPos, rotatedOffset));
  rightEye.visual.setRotation({0, 1, 0}, yRotationDegrees);

  rotatedOffset =
      Vector3RotateByAxisAngle(mouth.currentOffset, {0, 1, 0}, yRotation);
  mouth.visual.setPosition(Vector3Add(physicsPos, rotatedOffset));
  mouth.visual.setRotation({0, 1, 0}, yRotationDegrees);

  // Eyebrow positioning
  rotatedOffset =
      Vector3RotateByAxisAngle(leftEyebrow.currentOffset, {0, 1, 0}, yRotation);
  leftEyebrow.visual.setPosition(Vector3Add(physicsPos, rotatedOffset));
  leftEyebrow.visual.setRotation({0, 1, 0}, yRotationDegrees);

  rotatedOffset = Vector3RotateByAxisAngle(rightEyebrow.currentOffset,
                                           {0, 1, 0}, yRotation);
  rightEyebrow.visual.setPosition(Vector3Add(physicsPos, rotatedOffset));
  rightEyebrow.visual.setRotation({0, 1, 0}, yRotationDegrees);
}

// updateFacialFeatures() method removed - now integrated into
// updateVisualPositions() for efficiency

void HumanoidCharacter::applyMovement(const Vector3& direction, float speed,
                                      float forceMultiplier) {
  applyMovementInternal(direction, speed, forceMultiplier);
}

void HumanoidCharacter::applyMovementInternal(Vector3 movement, float speed,
                                              float forceMultiplier) {
  if (!characterBody) return;

  btVector3 currentVelocity = characterBody->getLinearVelocity();

  // INSTANT STOP when no input - NO MORE SLIDING!
  if (Vector3Length(movement) < 0.01f) {
    // STOP IMMEDIATELY - set horizontal velocity to zero
    btVector3 stopVelocity(0.0f, currentVelocity.getY(), 0.0f);
    characterBody->setLinearVelocity(stopVelocity);
    characterBody->setAngularVelocity(btVector3(0, 0, 0));
    return;
  }

  movement = Vector3Normalize(movement);

  // Better movement with collision sliding - preserve Y velocity for jumping
  btVector3 targetVelocity(movement.x * speed, currentVelocity.getY(),
                           movement.z * speed);

  // Apply movement force instead of setting velocity directly for better
  // collision response
  btVector3 velocityDiff = targetVelocity - currentVelocity;
  velocityDiff.setY(0);  // Don't interfere with jumping/gravity

  // Apply enhanced movement impulse using the force multiplier
  btVector3 movementImpulse =
      velocityDiff * GameSettings::Character::MASS * 0.1f * forceMultiplier;
  characterBody->applyCentralImpulse(movementImpulse);

  // Limit maximum horizontal speed to prevent sliding
  btVector3 vel = characterBody->getLinearVelocity();
  float horizontalSpeed =
      sqrt(vel.getX() * vel.getX() + vel.getZ() * vel.getZ());
  if (horizontalSpeed > speed) {
    float scale = speed / horizontalSpeed;
    characterBody->setLinearVelocity(
        btVector3(vel.getX() * scale, vel.getY(), vel.getZ() * scale));
  }

  updateRotationTowardsMovement(movement);
}

void HumanoidCharacter::updateRotationTowardsMovement(Vector3 movement) {
  if (!characterBody) return;

  if (Vector3Length(movement) <= 0.1f) {
    characterBody->setAngularVelocity(btVector3(0, 0, 0));
    return;
  }

  float targetRotation = atan2(movement.x, movement.z);

  btTransform transform;
  characterBody->getMotionState()->getWorldTransform(transform);
  btQuaternion currentRotation = transform.getRotation();

  float currentYRotation =
      atan2(2.0f * (currentRotation.getW() * currentRotation.getY() +
                    currentRotation.getX() * currentRotation.getZ()),
            1.0f - 2.0f * (currentRotation.getY() * currentRotation.getY() +
                           currentRotation.getZ() * currentRotation.getZ()));

  float angleDiff = targetRotation - currentYRotation;

  while (angleDiff > M_PI) angleDiff -= 2 * M_PI;
  while (angleDiff < -M_PI) angleDiff += 2 * M_PI;

  const float rotationSpeed = GameSettings::Character::TURN_SPEED * 0.05f;

  if (std::fabs(angleDiff) > 0.01f) {
    float targetAngularVelocity = angleDiff * rotationSpeed * 60.0f;
    characterBody->setAngularVelocity(btVector3(0, targetAngularVelocity, 0));
  } else {
    characterBody->setAngularVelocity(btVector3(0, 0, 0));
  }
}

void HumanoidCharacter::onRemovedFromWorld(GameWorld& world) {
  removeFromPhysics(world.getDynamicsWorld());
}

void HumanoidCharacter::moveTowards(Vector3 target, float deltaTime) {
  // Calculate direction to target
  Vector3 direction = Vector3Subtract(target, position);
  direction.y = 0;  // Only move on horizontal plane

  float distance = Vector3Length(direction);

  if (distance < 0.5f) {
    applyMovement({0, 0, 0}, 0.0f);
    return;
  }

  // Normalize direction and apply movement
  direction = Vector3Normalize(direction);
  float speed = GameSettings::Character::MOVEMENT_SPEED;

  applyMovement(direction, speed);
}

void HumanoidCharacter::jump() {
  if (!characterBody) {
    return;
  }

  if (jumpCooldown > 0.0f) {
    return;
  }

  // Check if character is on ground - balanced check
  bool onGround = isOnGround();

  if (!onGround) {
    return;  // Cannot jump if not on ground
  }

  // Additional check: Don't allow jumping if moving up too fast (already
  // jumping)
  btVector3 velocity = characterBody->getLinearVelocity();

  if (velocity.getY() > 2.0f) {
    return;  // Already moving up fast, don't allow another jump
  }

  // Jump force
  btVector3 jumpImpulse(0, GameSettings::Character::JUMP_IMPULSE * 2.0f, 0);
  characterBody->applyCentralImpulse(jumpImpulse);

  isJumping = true;
  jumpCooldown =
      0.2f;  // Moderate cooldown to prevent rapid spam but allow normal jumping
}

bool HumanoidCharacter::isOnGround() const {
  if (!characterBody || !physicsWorld) {
    return false;
  }

  // Raycast downward to check for ground
  btTransform transform;
  characterBody->getMotionState()->getWorldTransform(transform);
  btVector3 origin = transform.getOrigin();

  // Ground detection using raycast
  float groundContactOffset =
      GameSettings::CharacterCalculations::getGroundContactOffset();
  btVector3 from =
      origin + btVector3(0, groundContactOffset + 0.05f,
                         0);  // Start slightly above ground contact
  btVector3 to = origin + btVector3(0, groundContactOffset - 0.25f,
                                    0);  // Check below ground contact

  btCollisionWorld::ClosestRayResultCallback rayCallback(from, to);
  // Use the proper collision groups for raycast
  rayCallback.m_collisionFilterGroup =
      GameSettings::Collision::Groups::CHARACTER_TORSO;
  rayCallback.m_collisionFilterMask =
      GameSettings::Collision::Groups::WORLD_OBJECTS;  // Only hit world objects

  physicsWorld->rayTest(from, to, rayCallback);

  // Balanced velocity check - allow for small physics fluctuations but prevent
  // air jumps
  btVector3 velocity = characterBody->getLinearVelocity();
  bool velocityCheck =
      velocity.getY() < 1.0f && velocity.getY() > -1.0f;  // More reasonable

  bool hasHit = rayCallback.hasHit();
  bool result = hasHit && velocityCheck;

  return result;
}

Vector3 HumanoidCharacter::getFeetPosition() const {
  if (!characterBody) return position;

  btTransform transform;
  characterBody->getMotionState()->getWorldTransform(transform);
  btVector3 origin = transform.getOrigin();

  // Return feet position (ground contact point)
  return {origin.getX(),
          GameSettings::CharacterCalculations::getGroundYFromPhysicsCenterY(
              origin.getY()),
          origin.getZ()};
}

Vector3 HumanoidCharacter::getTorsoPosition() const {
  if (!characterBody) return position;

  btTransform transform;
  characterBody->getMotionState()->getWorldTransform(transform);
  btVector3 origin = transform.getOrigin();

  return {origin.getX(), origin.getY(), origin.getZ()};
}

void HumanoidCharacter::draw() const {
  // Draw all visual parts
  head.visual.draw();
  torso.visual.draw();
  leftArm.visual.draw();
  rightArm.visual.draw();
  leftLeg.visual.draw();
  rightLeg.visual.draw();

  // Draw facial features for character expression
  leftEye.visual.draw();
  rightEye.visual.draw();
  mouth.visual.draw();
  leftEyebrow.visual.draw();
  rightEyebrow.visual.draw();
}

BoundingBox HumanoidCharacter::getBoundingBox() const {
  Vector3 torsoPos = getTorsoPosition();
  Vector3 min = {torsoPos.x - 0.5f, torsoPos.y - 1.0f, torsoPos.z - 0.5f};
  Vector3 max = {torsoPos.x + 0.5f, torsoPos.y + 1.0f, torsoPos.z + 0.5f};
  return {min, max};
}

// Individual body part collision detection methods
Vector3 HumanoidCharacter::getPartWorldPosition(
    const HumanoidVisualPart& part) const {
  Vector3 characterPos = getPosition();

  // Get character rotation (yaw only for simplicity)
  btTransform transform;
  characterBody->getMotionState()->getWorldTransform(transform);
  btQuaternion rotation = transform.getRotation();
  float yaw = rotation.getAngle() * rotation.getAxis().getY();

  // Apply rotation to the part's current offset
  float cosYaw = cos(yaw);
  float sinYaw = sin(yaw);

  Vector3 rotatedOffset = {
      part.currentOffset.x * cosYaw - part.currentOffset.z * sinYaw,
      part.currentOffset.y,
      part.currentOffset.x * sinYaw + part.currentOffset.z * cosYaw};

  return Vector3Add(characterPos, rotatedOffset);
}

BoundingBox HumanoidCharacter::getPartBoundingBox(
    const HumanoidVisualPart& part) const {
  Vector3 partPos = getPartWorldPosition(part);
  Vector3 halfSize = Vector3Scale(part.visual.getSize(), 0.5f);

  Vector3 min = Vector3Subtract(partPos, halfSize);
  Vector3 max = Vector3Add(partPos, halfSize);

  return {min, max};
}

bool HumanoidCharacter::checkPartCollision(const HumanoidVisualPart& part,
                                           const BoundingBox& obstacle) const {
  BoundingBox partBox = getPartBoundingBox(part);
  return CheckCollisionBoxes(partBox, obstacle);
}

bool HumanoidCharacter::checkAnyPartCollision(
    const BoundingBox& obstacle) const {
  // Check all major body parts for collision
  if (checkPartCollision(head, obstacle)) return true;
  if (checkPartCollision(torso, obstacle)) return true;
  if (checkPartCollision(leftArm, obstacle)) return true;
  if (checkPartCollision(rightArm, obstacle)) return true;
  if (checkPartCollision(leftLeg, obstacle)) return true;
  if (checkPartCollision(rightLeg, obstacle)) return true;

  return false;
}

bool HumanoidCharacter::wouldCollideAfterMovement(Vector3 movement,
                                                  float deltaTime) const {
  if (!world) return false;  // No world to check against
  if (!characterBody) return false;

  // Calculate predicted position after movement
  Vector3 predictedMovement = {movement.x * deltaTime,
                               0,  // Don't predict Y movement for simplicity
                               movement.z * deltaTime};

  // Temporarily store current character position
  Vector3 originalPos = getPosition();

  // Check predicted positions against world obstacles
  Vector3 predictedPosition = Vector3Add(originalPos, predictedMovement);

  auto computePredictedBox = [&](const HumanoidVisualPart& part) {
    Vector3 offset = Vector3Add(part.currentOffset, predictedMovement);
    Vector3 center = Vector3Add(predictedPosition, offset);
    Vector3 halfSize = Vector3Scale(part.visual.getSize(), 0.5f);
    return BoundingBox{Vector3Subtract(center, halfSize),
                       Vector3Add(center, halfSize)};
  };

  const BoundingBox predictedHeadBox = computePredictedBox(head);
  const BoundingBox predictedTorsoBox = computePredictedBox(torso);
  const BoundingBox predictedLeftArmBox = computePredictedBox(leftArm);
  const BoundingBox predictedRightArmBox = computePredictedBox(rightArm);
  const BoundingBox predictedLeftLegBox = computePredictedBox(leftLeg);
  const BoundingBox predictedRightLegBox = computePredictedBox(rightLeg);

  // Check against world objects (simple implementation)
  // This is a basic collision prediction - in a full implementation,
  // you would iterate through all world objects and check collisions
  const auto& objects = world->getObjects();
  for (const auto& obj : objects) {
    if (obj.get() == this) continue;  // Skip self

    BoundingBox objBox = obj->getBoundingBox();

    // Check if any predicted part would collide
    if (CheckCollisionBoxes(predictedHeadBox, objBox) ||
        CheckCollisionBoxes(predictedTorsoBox, objBox) ||
        CheckCollisionBoxes(predictedLeftArmBox, objBox) ||
        CheckCollisionBoxes(predictedRightArmBox, objBox) ||
        CheckCollisionBoxes(predictedLeftLegBox, objBox) ||
        CheckCollisionBoxes(predictedRightLegBox, objBox)) {
      return true;
    }
  }

  return false;
}

// Multi-body physics system implementation
void HumanoidCharacter::createIndividualPhysicsBodies() {
  if (!physicsWorld || !characterBody) return;

  // Get character position for relative positioning
  btTransform characterTransform = characterBody->getWorldTransform();
  btVector3 characterPos = characterTransform.getOrigin();

  // Create physics bodies for each body part
  createIndividualPhysicsBody(
      headPhysics, {characterPos.x(), characterPos.y(), characterPos.z()});
  createIndividualPhysicsBody(
      torsoPhysics, {characterPos.x(), characterPos.y(), characterPos.z()});
  createIndividualPhysicsBody(
      leftArmPhysics, {characterPos.x(), characterPos.y(), characterPos.z()});
  createIndividualPhysicsBody(
      rightArmPhysics, {characterPos.x(), characterPos.y(), characterPos.z()});
  createIndividualPhysicsBody(
      leftLegPhysics, {characterPos.x(), characterPos.y(), characterPos.z()});
  createIndividualPhysicsBody(
      rightLegPhysics, {characterPos.x(), characterPos.y(), characterPos.z()});
}

void HumanoidCharacter::createIndividualPhysicsBody(HumanoidPhysicsPart& part,
                                                    Vector3 worldPosition) {
  if (!physicsWorld) return;

  // Create box shape for the body part
  btVector3 halfExtents(part.size.x / 2.0f, part.size.y / 2.0f,
                        part.size.z / 2.0f);
  part.shape = new btBoxShape(halfExtents);
  part.shape->setMargin(GameSettings::Collision::SHAPE_MARGIN);

  // Calculate world position for this part
  Vector3 partWorldPos = Vector3Add(worldPosition, part.baseOffset);

  // Create transform
  btTransform transform;
  transform.setIdentity();
  transform.setOrigin(
      btVector3(partWorldPos.x, partWorldPos.y, partWorldPos.z));

  // Create motion state
  part.motionState = new btDefaultMotionState(transform);

  // Create rigid body with very low mass (kinematic-like behavior)
  float mass = 0.1f;  // Very light so they don't interfere with main movement
  btVector3 inertia(0, 0, 0);
  part.shape->calculateLocalInertia(mass, inertia);

  btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, part.motionState,
                                                  part.shape, inertia);
  rbInfo.m_friction = GameSettings::Collision::FRICTION;
  rbInfo.m_rollingFriction = GameSettings::Collision::ROLLING_FRICTION;
  rbInfo.m_restitution = GameSettings::Collision::RESTITUTION;

  part.body = new btRigidBody(rbInfo);

  // Set as kinematic for controlled movement
  part.body->setCollisionFlags(part.body->getCollisionFlags() |
                               btCollisionObject::CF_KINEMATIC_OBJECT);
  part.body->setActivationState(DISABLE_DEACTIVATION);

  // Add to physics world with proper collision filtering
  // Individual body parts should collide with world objects only
  physicsWorld->addRigidBody(part.body, part.collisionGroup,
                             GameSettings::Collision::Groups::WORLD_OBJECTS);
}

void HumanoidCharacter::updatePhysicsBodyPosition(HumanoidPhysicsPart& part,
                                                  Vector3 worldPosition) {
  if (!part.body) return;

  // Calculate new world position for this part
  Vector3 partWorldPos = Vector3Add(worldPosition, part.baseOffset);

  // Get character rotation to apply to physics bodies too
  btTransform characterTransform;
  characterBody->getMotionState()->getWorldTransform(characterTransform);
  btQuaternion characterRotation = characterTransform.getRotation();

  // Update the physics body position AND rotation
  btTransform transform;
  transform.setIdentity();
  transform.setOrigin(
      btVector3(partWorldPos.x, partWorldPos.y, partWorldPos.z));
  transform.setRotation(characterRotation);  // Apply same rotation as character

  part.body->setWorldTransform(transform);
  part.body->getMotionState()->setWorldTransform(transform);
}

void HumanoidCharacter::synchronizeVisualWithPhysics() {
  if (!characterBody) return;

  // Get main character position
  btTransform characterTransform = characterBody->getWorldTransform();
  btVector3 characterPos = characterTransform.getOrigin();
  Vector3 worldPos = {characterPos.x(), characterPos.y(), characterPos.z()};

  // Update each physics body to follow the visual parts
  updatePhysicsBodyPosition(headPhysics, worldPos);
  updatePhysicsBodyPosition(torsoPhysics, worldPos);
  updatePhysicsBodyPosition(leftArmPhysics, worldPos);
  updatePhysicsBodyPosition(rightArmPhysics, worldPos);
  updatePhysicsBodyPosition(leftLegPhysics, worldPos);
  updatePhysicsBodyPosition(rightLegPhysics, worldPos);
}

void HumanoidCharacter::removeAllPhysicsBodies() {
  if (!physicsWorld) return;

  // Remove all individual physics bodies from the world
  if (headPhysics.body) {
    physicsWorld->removeRigidBody(headPhysics.body);
    headPhysics.cleanup();
  }
  if (torsoPhysics.body) {
    physicsWorld->removeRigidBody(torsoPhysics.body);
    torsoPhysics.cleanup();
  }
  if (leftArmPhysics.body) {
    physicsWorld->removeRigidBody(leftArmPhysics.body);
    leftArmPhysics.cleanup();
  }
  if (rightArmPhysics.body) {
    physicsWorld->removeRigidBody(rightArmPhysics.body);
    rightArmPhysics.cleanup();
  }
  if (leftLegPhysics.body) {
    physicsWorld->removeRigidBody(leftLegPhysics.body);
    leftLegPhysics.cleanup();
  }
  if (rightLegPhysics.body) {
    physicsWorld->removeRigidBody(rightLegPhysics.body);
    rightLegPhysics.cleanup();
  }
}

void HumanoidCharacter::constrainBodyPartsToCharacter() {
  // This method can be used to add constraints between body parts
  // For now, we use kinematic bodies that follow the main character
  // In future implementations, you could add spring constraints
  // or other joint types to create more physics interactions
}

std::unique_ptr<GameObject> HumanoidCharacter::clone() const {
  return std::make_unique<HumanoidCharacter>(*this);
}

PhysicsBodyConfig HumanoidCharacter::getPhysicsConfig() const {
  PhysicsBodyConfig config;
  config.usesPhysics = false;
  return config;
}

void HumanoidCharacter::configurePhysicsBody(btRigidBody&) const {}
