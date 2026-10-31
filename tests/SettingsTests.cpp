#include <cmath>
#include <iostream>

#include "common/TestAssert.h"
#include "settings/Physics.h"

namespace {
bool almostEqual(float a, float b, float eps = 0.0001f) {
  return std::fabs(a - b) <= eps;
}
}  // namespace

int main() {
  using namespace GameSettings;

  const float groundSamples[] = {-5.0f, -1.0f, 0.0f, 1.0f, 10.0f};
  for (float groundY : groundSamples) {
    float centerY = CharacterCalculations::getPhysicsCenterYFromGroundY(groundY);
    float roundTripGroundY =
        CharacterCalculations::getGroundYFromPhysicsCenterY(centerY);
    TEST_ASSERT(almostEqual(groundY, roundTripGroundY),
                "Character ground/center conversion must be reversible");
  }

  TEST_ASSERT(Character::HEIGHT > 0.0f, "Character height must be positive");
  TEST_ASSERT(Character::RADIUS > 0.0f, "Character radius must be positive");
  TEST_ASSERT(Character::MAX_SPEED >= Character::MOVEMENT_SPEED,
              "Max speed should be >= base movement speed");

  TEST_ASSERT(BodyParts::HEAD_SIZE.x > 0.0f && BodyParts::HEAD_SIZE.y > 0.0f &&
                  BodyParts::HEAD_SIZE.z > 0.0f,
              "Head dimensions must be positive");
  TEST_ASSERT(BodyParts::TORSO_SIZE.x > 0.0f &&
                  BodyParts::TORSO_SIZE.y > 0.0f &&
                  BodyParts::TORSO_SIZE.z > 0.0f,
              "Torso dimensions must be positive");
  TEST_ASSERT(BodyParts::ARM_SIZE.x > 0.0f && BodyParts::ARM_SIZE.y > 0.0f &&
                  BodyParts::ARM_SIZE.z > 0.0f,
              "Arm dimensions must be positive");
  TEST_ASSERT(BodyParts::LEG_SIZE.x > 0.0f && BodyParts::LEG_SIZE.y > 0.0f &&
                  BodyParts::LEG_SIZE.z > 0.0f,
              "Leg dimensions must be positive");

  TEST_ASSERT(NPC::LOOKING_AT_SHELF_TIME > 0.0f,
              "Shelf looking time must be positive");
  TEST_ASSERT(NPC::MAX_SHOPPING_TIME > NPC::LOOKING_AT_SHELF_TIME,
              "Shopping time should be longer than a single shelf look");
  TEST_ASSERT(NPC::STUCK_TIMEOUT > 0.0f, "Stuck timeout must be positive");
  TEST_ASSERT(NPC::LEAVE_PROBABILITY >= 0.0f && NPC::LEAVE_PROBABILITY <= 1.0f,
              "Leave probability must be in [0, 1]");

  std::cout << "SettingsTests passed" << std::endl;
  return 0;
}

