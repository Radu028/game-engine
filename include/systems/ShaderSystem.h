#pragma once

#include "raylib.h"

class ShaderSystem {
 private:
  static ShaderSystem* instance;
  Shader lightingShader;
  bool initialized;

  // Sun lighting properties
  Vector3 sunDirection;
  Vector3 sunColor;
  float ambientStrength;

  // Cached uniform locations for better performance
  int sunDirectionLoc;
  int sunColorLoc;
  int ambientStrengthLoc;
  int timeLoc;

  // Time-based effects
  float gameTime;

  ShaderSystem();

 public:
  static ShaderSystem* getInstance();
  ~ShaderSystem();

  bool initialize();
  void cleanup();

  // Shader management
  void beginShaderMode();
  void endShaderMode();

  // Sun lighting control
  void setSunDirection(Vector3 direction);
  void setSunColor(Vector3 color);
  void setAmbientStrength(float strength);
  void updateUniforms(const Camera3D& camera);
  void updateTime(float deltaTime);

  // Dynamic lighting effects
  void enableDynamicSun(bool enable = true);
  void setTimeOfDay(
      float normalizedTime);  // 0.0 = midnight, 0.5 = noon, 1.0 = midnight

  Shader getShader() const { return lightingShader; }
  Vector3 getSunDirection() const { return sunDirection; }
  bool isInitialized() const { return initialized; }
};
