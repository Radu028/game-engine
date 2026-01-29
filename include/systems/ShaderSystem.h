#pragma once

#include "raylib.h"

class ShaderSystem {
 private:
  static ShaderSystem* instance;
  Shader lightingShader;

  // Sun lighting properties
  Vector3 sunDirection;
  Vector3 sunColor;
  float ambientStrength;

  // Cached uniform locations for better performance
  int sunDirectionLoc;
  int sunColorLoc;
  int ambientStrengthLoc;

  ShaderSystem();

 public:
  static ShaderSystem* getInstance();
  static void destroyInstance();
  ~ShaderSystem();

  ShaderSystem(const ShaderSystem&) = delete;
  ShaderSystem& operator=(const ShaderSystem&) = delete;

  void initialize();
  void cleanup();

  // Shader management
  void beginShaderMode();
  void endShaderMode();

  void updateUniforms(const Camera3D& camera);

  Shader getShader() const { return lightingShader; }
};
