#include "systems/ShaderSystem.h"

#include <cmath>
#include <iostream>

#include "raylib.h"
#include "raymath.h"

#ifndef PI
#define PI 3.14159265358979323846f
#endif

ShaderSystem* ShaderSystem::instance = nullptr;

ShaderSystem::ShaderSystem()
    : initialized(false),
      sunDirection({-0.3f, -1.0f, -0.4f}),
      sunColor({1.0f, 0.9f, 0.7f}),
      ambientStrength(0.5f),
      sunDirectionLoc(-1),
      sunColorLoc(-1),
      ambientStrengthLoc(-1),
      timeLoc(-1),
      gameTime(0.0f) {}

ShaderSystem* ShaderSystem::getInstance() {
  if (instance == nullptr) {
    instance = new ShaderSystem();
  }
  return instance;
}

ShaderSystem::~ShaderSystem() { cleanup(); }

bool ShaderSystem::initialize() {
  if (initialized) return true;

  lightingShader =
      LoadShader("../resources/lighting.vs", "../resources/lighting.fs");

  if (lightingShader.id == 0) {
    std::cout << "Warning: Failed to load custom shaders, using default shader"
              << std::endl;
    return false;
  }

  lightingShader.locs[SHADER_LOC_MATRIX_MVP] =
      GetShaderLocation(lightingShader, "mvp");
  lightingShader.locs[SHADER_LOC_MATRIX_VIEW] =
      GetShaderLocation(lightingShader, "matView");
  lightingShader.locs[SHADER_LOC_MATRIX_PROJECTION] =
      GetShaderLocation(lightingShader, "matProjection");
  lightingShader.locs[SHADER_LOC_MATRIX_MODEL] =
      GetShaderLocation(lightingShader, "matModel");
  lightingShader.locs[SHADER_LOC_VECTOR_VIEW] =
      GetShaderLocation(lightingShader, "viewPos");

  sunDirectionLoc = GetShaderLocation(lightingShader, "sunDirection");
  sunColorLoc = GetShaderLocation(lightingShader, "sunColor");
  ambientStrengthLoc = GetShaderLocation(lightingShader, "ambientStrength");
  timeLoc = GetShaderLocation(lightingShader, "time");

  SetShaderValue(lightingShader, sunDirectionLoc, &sunDirection,
                 SHADER_UNIFORM_VEC3);
  SetShaderValue(lightingShader, sunColorLoc, &sunColor, SHADER_UNIFORM_VEC3);
  SetShaderValue(lightingShader, ambientStrengthLoc, &ambientStrength,
                 SHADER_UNIFORM_FLOAT);
  SetShaderValue(lightingShader, timeLoc, &gameTime, SHADER_UNIFORM_FLOAT);

  initialized = true;
  std::cout << "Shader system initialized successfully" << std::endl;
  return true;
}

void ShaderSystem::cleanup() {
  if (initialized) {
    UnloadShader(lightingShader);
    initialized = false;
  }
}

void ShaderSystem::beginShaderMode() {
  if (initialized) {
    BeginShaderMode(lightingShader);
  }
}

void ShaderSystem::endShaderMode() {
  if (initialized) {
    EndShaderMode();
  }
}

void ShaderSystem::setSunDirection(Vector3 direction) {
  if (Vector3Length(direction) < 0.0001f) {
    direction = (Vector3){0.0f, -1.0f, 0.0f};
  }

  sunDirection = Vector3Normalize(direction);
  if (initialized && sunDirectionLoc != -1) {
    SetShaderValue(lightingShader, sunDirectionLoc, &sunDirection,
                   SHADER_UNIFORM_VEC3);
  }
}

void ShaderSystem::setSunColor(Vector3 color) {
  sunColor = color;
  if (initialized && sunColorLoc != -1) {
    SetShaderValue(lightingShader, sunColorLoc, &sunColor, SHADER_UNIFORM_VEC3);
  }
}

void ShaderSystem::setAmbientStrength(float strength) {
  ambientStrength = strength;
  if (initialized && ambientStrengthLoc != -1) {
    SetShaderValue(lightingShader, ambientStrengthLoc, &ambientStrength,
                   SHADER_UNIFORM_FLOAT);
  }
}

void ShaderSystem::updateUniforms(const Camera3D& camera) {
  if (!initialized) return;

  // Update view position for specular calculations
  SetShaderValue(lightingShader, lightingShader.locs[SHADER_LOC_VECTOR_VIEW],
                 &camera.position, SHADER_UNIFORM_VEC3);
}

void ShaderSystem::updateTime(float deltaTime) {
  if (!initialized) return;

  gameTime += deltaTime;
  if (timeLoc != -1) {
    SetShaderValue(lightingShader, timeLoc, &gameTime, SHADER_UNIFORM_FLOAT);
  }
}

void ShaderSystem::enableDynamicSun(bool enable) {
  if (!enable) return;

  // Create a dynamic sun that moves throughout the day
  float timeOfDay =
      fmod(gameTime * 0.1f, 1.0f);  // One full cycle every 10 seconds for demo
  setTimeOfDay(timeOfDay);
}

void ShaderSystem::setTimeOfDay(float normalizedTime) {
  if (!initialized) return;

  // normalizedTime: 0.0 = midnight, 0.25 = sunrise, 0.5 = noon, 0.75 =
  // sunset, 1.0 = midnight
  float angle = normalizedTime * 2.0f * PI;

  // Calculate sun position (circular path)
  Vector3 newSunDirection = {
      sin(angle) * 0.8f,  // X component
      -cos(angle),        // Y component (negative for downward)
      -0.4f               // Z component (constant)
  };

  // Calculate sun color based on time of day
  Vector3 newSunColor;
  if (normalizedTime < 0.25f || normalizedTime > 0.75f) {
    // Night time - darker, bluish light
    float nightIntensity =
        0.1f + 0.2f * (1.0f - abs(normalizedTime - 0.5f) * 2.0f);
    newSunColor = (Vector3){0.2f * nightIntensity, 0.3f * nightIntensity,
                            0.8f * nightIntensity};
  } else if (normalizedTime >= 0.2f && normalizedTime <= 0.3f) {
    // Sunrise - warm orange/red
    newSunColor = (Vector3){1.0f, 0.5f, 0.2f};
  } else if (normalizedTime >= 0.7f && normalizedTime <= 0.8f) {
    // Sunset - warm orange/red
    newSunColor = (Vector3){1.0f, 0.4f, 0.1f};
  } else {
    // Day time - bright white/yellow
    newSunColor = (Vector3){1.0f, 0.95f, 0.8f};
  }

  setSunDirection(newSunDirection);
  setSunColor(newSunColor);

  // Adjust ambient strength based on time of day
  float newAmbientStrength = 0.15f + 0.35f * (0.5f + 0.5f * cos(angle));
  setAmbientStrength(newAmbientStrength);
}
