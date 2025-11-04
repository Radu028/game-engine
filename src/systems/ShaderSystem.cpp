#include "systems/ShaderSystem.h"

#include <iostream>

#include "raylib.h"

ShaderSystem* ShaderSystem::instance = nullptr;

ShaderSystem::ShaderSystem()
    : initialized(false),
      sunDirection({-0.3f, -1.0f, -0.4f}),
      sunColor({1.0f, 0.9f, 0.7f}),
      ambientStrength(0.5f),
      sunDirectionLoc(-1),
      sunColorLoc(-1),
      ambientStrengthLoc(-1) {}

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
  SetShaderValue(lightingShader, sunDirectionLoc, &sunDirection,
                 SHADER_UNIFORM_VEC3);
  SetShaderValue(lightingShader, sunColorLoc, &sunColor, SHADER_UNIFORM_VEC3);
  SetShaderValue(lightingShader, ambientStrengthLoc, &ambientStrength,
                 SHADER_UNIFORM_FLOAT);

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

void ShaderSystem::updateUniforms(const Camera3D& camera) {
  if (!initialized) return;

  // Update view position for specular calculations
  SetShaderValue(lightingShader, lightingShader.locs[SHADER_LOC_VECTOR_VIEW],
                 &camera.position, SHADER_UNIFORM_VEC3);
}
