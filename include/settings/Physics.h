#pragma once
#include <btBulletDynamicsCommon.h>

#include "raylib.h"

// Using namespace instead of struct for better organization and extensibility
namespace GameSettings {
// Physics constants
namespace Physics {
static constexpr float GRAVITY_ACCELERATION =
    -30.0f;  // Gravity setting for less floaty jumps
static constexpr float FIXED_TIMESTEP = 1.0f / 60.0f;  // 60 FPS simulation
static constexpr int MAX_SUBSTEPS = 10;  // Maximum physics substeps per frame
}  // namespace Physics

// Player/Character constants
namespace Character {
static constexpr float HEIGHT = 1.8f;
static constexpr float RADIUS = 0.3f;
static constexpr float MASS = 70.0f;
static constexpr float MOVEMENT_FORCE = 1500.0f;
static constexpr float JUMP_IMPULSE = 400.0f;  // Jump power setting
static constexpr float MAX_SPEED = 8.0f;
static constexpr float DAMPING_LINEAR = 0.85f;  // Reduced for smoother movement
static constexpr float DAMPING_ANGULAR = 0.8f;
static constexpr float MOVEMENT_SPEED = 5.0f;
static constexpr float TURN_SPEED =
    1.0f;  // How fast player turns toward movement direction
}  // namespace Character

// Camera constants
namespace Camera {
static const Vector3 OFFSET = {0.0f, 3.0f, 8.0f};
static constexpr float TARGET_Y_OFFSET = 1.5f;
}  // namespace Camera

// Body part proportions for blocky character style
namespace BodyParts {
// Head proportions (cubic/blocky style)
static const Vector3 HEAD_SIZE = {0.5f, 0.5f, 0.5f};
static const Color HEAD_COLOR = YELLOW;

// Torso proportions (brick-like proportions)
static const Vector3 TORSO_SIZE = {0.7f, 0.9f, 0.4f};
static const Color TORSO_COLOR = BLUE;

// Arms proportions (proportional to torso)
static const Vector3 ARM_SIZE = {0.25f, 0.6f, 0.25f};
static const Color ARM_COLOR = YELLOW;

// Legs proportions (proportional and stable)
static const Vector3 LEG_SIZE = {0.3f, 0.7f, 0.3f};
static const Color LEG_COLOR = RED;

// Positioning offsets for character proportions
static constexpr float ARM_OFFSET_X = 0.48f;  // Distance from center to arms
static constexpr float LEG_OFFSET_X = 0.18f;  // Distance from center to legs
static constexpr float HEAD_OFFSET_Y =
    0.05f;  // Additional height for head positioning
}  // namespace BodyParts

// Animation constants
namespace Animation {
static constexpr float SPEED_THRESHOLD = 0.25f;
static constexpr float SPEED_MULTIPLIER = 2.0f;
static constexpr float ARM_SWING_AMOUNT = 20.0f;
static constexpr float LEG_SWING_AMOUNT = 25.0f;
}  // namespace Animation

// Physics collision constants
namespace Collision {
static constexpr float GROUND_CHECK_DISTANCE = 0.2f;
static constexpr float GROUND_CHECK_TOLERANCE = 0.20f;
static constexpr float VELOCITY_Y_THRESHOLD = 2.0f;
static constexpr float SHAPE_MARGIN = 0.01f;
static constexpr float FRICTION =
    0.4f;  // Reduced friction for better sliding along walls
static constexpr float ROLLING_FRICTION = 0.2f;  // Reduced rolling friction
static constexpr float RESTITUTION = 0.0f;

// Collision groups for multi-body character physics
namespace Groups {
static constexpr short WORLD_OBJECTS = 1;    // Floor, walls, obstacles
static constexpr short CHARACTER_TORSO = 2;  // Main character body
static constexpr short CHARACTER_HEAD = 4;   // Character head
static constexpr short CHARACTER_ARMS = 8;   // Character arms
static constexpr short CHARACTER_LEGS = 16;  // Character legs

// Collision masks - what each group can collide with
static constexpr short WORLD_MASK =
    CHARACTER_TORSO | CHARACTER_HEAD | CHARACTER_ARMS | CHARACTER_LEGS;
static constexpr short CHARACTER_MASK =
    WORLD_OBJECTS;  // Character parts only collide with world

// All character parts combined for easy reference
static constexpr short ALL_CHARACTER_PARTS =
    CHARACTER_TORSO | CHARACTER_HEAD | CHARACTER_ARMS | CHARACTER_LEGS;
}  // namespace Groups
}  // namespace Collision

// Utility functions for character positioning
namespace CharacterCalculations {
// Calculate total physics capsule height (height + 2*radius)
inline float getTotalCapsuleHeight() {
  return Character::HEIGHT + 2 * Character::RADIUS;
}

// Calculate half of total capsule height for positioning
inline float getHalfCapsuleHeight() { return getTotalCapsuleHeight() / 2.0f; }

// Calculate the leg base Y offset based on leg size
inline float getLegBaseYOffset() {
  // Legs should be positioned so their bottom aligns with physics body bottom
  // Physics body bottom is at: physicsCenter - halfCapsuleHeight
  // Leg visual center should be at: legBottom + legHeight/2
  // Therefore: legCenterOffset = -halfCapsuleHeight + legHeight/2
  return -getHalfCapsuleHeight() + BodyParts::LEG_SIZE.y / 2.0f;
}

// Calculate torso base Y offset
inline float getTorsoBaseYOffset() {
  // Torso should be positioned above the legs
  float legTop = getLegBaseYOffset() + BodyParts::LEG_SIZE.y / 2.0f;
  return legTop + BodyParts::TORSO_SIZE.y / 2.0f;
}

// Calculate head base Y offset
inline float getHeadBaseYOffset() {
  // Head should be positioned above the torso
  float torsoTop = getTorsoBaseYOffset() + BodyParts::TORSO_SIZE.y / 2.0f;
  return torsoTop + BodyParts::HEAD_SIZE.y / 2.0f + BodyParts::HEAD_OFFSET_Y;
}

// Calculate arm base Y offset
inline float getArmBaseYOffset() {
  // Arms should be positioned at torso level
  return getTorsoBaseYOffset() +
         BodyParts::TORSO_SIZE.y * 0.3f;  // Slightly above torso center
}

// Calculate ground contact point Y offset from physics center
inline float getGroundContactOffset() { return -getHalfCapsuleHeight(); }

// Calculate the Y position where physics body center should be placed
// given a desired ground contact Y position
inline float getPhysicsCenterYFromGroundY(float groundY) {
  return groundY + getHalfCapsuleHeight();
}

// Calculate the ground contact Y position from physics body center Y
inline float getGroundYFromPhysicsCenterY(float physicsCenterY) {
  return physicsCenterY - getHalfCapsuleHeight();
}
}  // namespace CharacterCalculations

// NPC AI constants
namespace NPC {
static constexpr float LOOKING_AT_SHELF_TIME = 10.0f;
static constexpr float CHAT_INTERVAL = 4.0f;
static constexpr float ARRIVAL_THRESHOLD = 2.5f;
static constexpr float MAX_SHOPPING_TIME = 90.0f;
static constexpr float MAX_WANDER_TIME = 5.0f;
static constexpr float LEAVE_PROBABILITY = 0.1f;
static constexpr float STUCK_THRESHOLD = 0.02f;
static constexpr float STUCK_TIMEOUT = 7.0f;
static constexpr float EXIT_DISTANCE = 2.0f;
static constexpr float SHOP_EXIT_DISTANCE = 15.0f;
static constexpr int MIN_SHELVES_TO_VISIT = 2;
}  // namespace NPC
}  // namespace GameSettings