#pragma once

#include <memory>

#include "raylib.h"
#include "rendering/Renderer.h"

class GameManager;
class GameWorld;
class ShaderSystem;
class HumanoidCharacter;
class PlayerController;

class Application {
 public:
  Application();
  ~Application();

  int run();

 private:
  Renderer renderer;
  Camera3D camera;
  float cameraDistance;
  float cameraAngleX;
  float cameraAngleY;

  ShaderSystem* shaderSystem;
  GameWorld* world;
  GameManager* gameManager;
  std::shared_ptr<HumanoidCharacter> playerCharacter;
  std::unique_ptr<PlayerController> playerController;

  void initialize();
  void processInput(float deltaTime);
  void update(float deltaTime);
  void render();
  void updateCameraTarget();
  void updateCameraPosition();
};
