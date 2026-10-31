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

// Body part proportions for blocky Roblox-style character
namespace BodyParts {
// Head proportions (cubic/blocky style - Roblox heads are bigger)
static const Vector3 HEAD_SIZE = {0.6f, 0.6f, 0.6f};
static const Color HEAD_COLOR = {255, 204, 153, 255};  // Skin tone

// Torso proportions (brick-like Roblox torso)
static const Vector3 TORSO_SIZE = {0.8f, 1.0f, 0.4f};
static const Color TORSO_COLOR = {0, 162, 232, 255};  // Bright blue shirt

// Arms proportions (Roblox arms)
static const Vector3 ARM_SIZE = {0.25f, 0.8f, 0.25f};
static const Color ARM_COLOR = {255, 204, 153, 255};  // Skin tone

// Legs proportions (Roblox legs)
static const Vector3 LEG_SIZE = {0.35f, 0.8f, 0.35f};
static const Color LEG_COLOR = {34, 139, 34, 255};  // Green pants

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
static constexpr float LOOKING_AT_SHELF_TIME =
    3.0f;  // Reduced for faster shopping
static constexpr float CHAT_INTERVAL = 5.0f;
static constexpr float ARRIVAL_THRESHOLD = 2.0f;
static constexpr float MAX_SHOPPING_TIME = 60.0f;
static constexpr float MAX_WANDER_TIME = 4.0f;
static constexpr float LEAVE_PROBABILITY =
    0.15f;  // Slightly more likely to leave
static constexpr float STUCK_THRESHOLD =
    0.015f;                                   // More sensitive stuck detection
static constexpr float STUCK_TIMEOUT = 5.0f;  // Faster response to being stuck
static constexpr float EXIT_DISTANCE = 2.5f;
static constexpr float SHOP_EXIT_DISTANCE = 12.0f;
static constexpr int MIN_SHELVES_TO_VISIT = 1;  // Reduced - buy faster
}  // namespace NPC
}  // namespace GameSettings