#include "systems/ShaderSystem.h"

#include <filesystem>
#include <sstream>
#include <string>
#include <vector>

#include "exceptions/GameExceptions.h"
#include "raylib.h"

ShaderSystem* ShaderSystem::instance = nullptr;

namespace {
namespace fs = std::filesystem;

std::string resolveResourcePathOrThrow(const std::string& relativePath) {
  const fs::path rel(relativePath);
  const fs::path cwd = fs::current_path();

  const std::vector<fs::path> candidates = {
      cwd / rel,
      cwd / ".." / rel,
      cwd / "../.." / rel,
      cwd / "../../.." / rel,
  };

  for (const auto& p : candidates) {
    std::error_code ec;
    if (fs::exists(p, ec) && !ec) {
      // Prefer absolute path to avoid raylib resolving relative paths
      // differently depending on working directory.
      return fs::absolute(p, ec).string();
    }
  }

  std::ostringstream oss;
  oss << "Resource not found: " << relativePath << "\nTried:";
  for (const auto& p : candidates) {
    oss << "\n - " << p.string();
  }
  throw GameInitException(oss.str());
}
}  // namespace

ShaderSystem::ShaderSystem()
    : sunDirection({-0.3f, -1.0f, -0.4f}),
      sunColor({1.0f, 0.9f, 0.7f}),
      ambientStrength(0.5f),
      sunDirectionLoc(-1),
      sunColorLoc(-1),
      ambientStrengthLoc(-1) {
  initialize();
}

ShaderSystem* ShaderSystem::getInstance() {
  if (instance == nullptr) {
    instance = new ShaderSystem();
  }
  return instance;
}

void ShaderSystem::destroyInstance() {
  delete instance;
  instance = nullptr;
}

ShaderSystem::~ShaderSystem() { cleanup(); }

void ShaderSystem::initialize() {
  const std::string vertexPath =
      resolveResourcePathOrThrow("resources/lighting.vs");
  const std::string fragmentPath =
      resolveResourcePathOrThrow("resources/lighting.fs");

  lightingShader = LoadShader(vertexPath.c_str(), fragmentPath.c_str());

  if (lightingShader.id == 0) {
    throw GameInitException("Failed to load custom shaders");
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

  // Validate shader uniform locations
  if (sunDirectionLoc == -1 || sunColorLoc == -1 || ambientStrengthLoc == -1) {
    throw GameInitException(
        "Failed to get shader uniform locations for lighting parameters");
  }

  SetShaderValue(lightingShader, sunDirectionLoc, &sunDirection,
                 SHADER_UNIFORM_VEC3);
  SetShaderValue(lightingShader, sunColorLoc, &sunColor, SHADER_UNIFORM_VEC3);
  SetShaderValue(lightingShader, ambientStrengthLoc, &ambientStrength,
                 SHADER_UNIFORM_FLOAT);
}

void ShaderSystem::cleanup() {
  if (lightingShader.id != 0) {
    UnloadShader(lightingShader);
    lightingShader = {};
  }
}

void ShaderSystem::beginShaderMode() { BeginShaderMode(lightingShader); }

void ShaderSystem::endShaderMode() { EndShaderMode(); }

void ShaderSystem::updateUniforms(const Camera3D& camera) {
  // Update view position for specular calculations
  SetShaderValue(lightingShader, lightingShader.locs[SHADER_LOC_VECTOR_VIEW],
                 &camera.position, SHADER_UNIFORM_VEC3);
}
