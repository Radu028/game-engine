#include "app/Application.h"

#include <algorithm>
#include <cmath>

#include "GameWorld.h"
#include "controllers/PlayerController.h"
#include "entities/HumanoidCharacter.h"
#include "exceptions/GameExceptions.h"
#include "game/GameManager.h"
#include "objects/Floor.h"
#include "settings/Physics.h"
#include "systems/InputSystem.h"
#include "systems/ShaderSystem.h"

namespace {
constexpr int kWindowWidth = 1280;
constexpr int kWindowHeight = 720;
constexpr char kWindowTitle[] = "3D Game";
constexpr int kTargetFPS = 120;
constexpr float kMouseSensitivity = 0.5f;
constexpr float kMinCameraDistance = 5.0f;
constexpr float kMaxCameraDistance = 50.0f;
constexpr float kMaxVerticalAngle = 89.0f;
constexpr float kMinVerticalAngle = -89.0f;
constexpr float kScrollScale = 2.0f;
}  // namespace

Application::Application()
    : camera{},
      cameraDistance(25.0f),
      cameraAngleX(0.0f),
      cameraAngleY(0.0f),
      shaderSystem(nullptr),
      world(nullptr),
      gameManager(nullptr) {}

Application::~Application() {
  GameManager::destroyInstance();
  GameWorld::destroyInstance();
}

int Application::run() {
  try {
    initialize();

    while (!renderer.shouldClose()) {
      const float deltaTime = renderer.frameTime();
      processInput(deltaTime);
      update(deltaTime);
      render();
    }

    return 0;
  } catch (const GameInitException& e) {
    TraceLog(LOG_ERROR, "Game Initialization Error: %s", e.what());
    return -1;
  } catch (const ResourceException& e) {
    TraceLog(LOG_ERROR, "Resource Error: %s", e.what());
    return -2;
  } catch (const PhysicsException& e) {
    TraceLog(LOG_ERROR, "Physics Error: %s", e.what());
    return -3;
  } catch (const AIException& e) {
    TraceLog(LOG_ERROR, "AI Error: %s", e.what());
    return -4;
  } catch (const GameException& e) {
    TraceLog(LOG_ERROR, "Game Error: %s", e.what());
    return -5;
  } catch (const std::exception& e) {
    TraceLog(LOG_ERROR, "Standard Library Error: %s", e.what());
    return -6;
  } catch (...) {
    TraceLog(LOG_ERROR, "Unknown error occurred!");
    return -7;
  }
}

void Application::initialize() {
  renderer.initialize(kWindowWidth, kWindowHeight, kWindowTitle);
  renderer.setTargetFPS(kTargetFPS);
  EnableCursor();

  shaderSystem = ShaderSystem::getInstance();

  camera = {{0.0f, 5.0f, 5.0f},
            {0.0f, 0.0f, 0.0f},
            {0.0f, 1.0f, 0.0f},
            45.0f,
            CAMERA_PERSPECTIVE};

  world = GameWorld::getInstance();

  playerCharacter =
      std::make_shared<HumanoidCharacter>(Vector3{0.0f, 1.0f, 15.0f}, world);
  world->addObject(playerCharacter);
  playerController = std::make_unique<PlayerController>(playerCharacter);
  if (!playerController) {
    throw GameInitException("Failed to create player controller");
  }

  world->addObject(std::make_shared<Floor>(Vector3{0.0f, 0.0f, 0.0f},
                                           Vector3{50.0f, 1.0f, 50.0f},
                                           DARKGREEN, true));

  gameManager = GameManager::getInstance();
}

void Application::processInput(float /*deltaTime*/) {
  InputSystem::updateMouseCamera();

  if (InputSystem::isRightMouseDown()) {
    const Vector2 mouseDelta = InputSystem::getMouseDelta();
    cameraAngleX -= mouseDelta.x * kMouseSensitivity;
    cameraAngleY += mouseDelta.y * kMouseSensitivity;
    cameraAngleY =
        std::clamp(cameraAngleY, kMinVerticalAngle, kMaxVerticalAngle);
  }

  const float wheelMove = GetMouseWheelMove();
  if (wheelMove != 0.0f) {
    const float desiredDistance = cameraDistance - wheelMove * kScrollScale;
    cameraDistance =
        std::clamp(desiredDistance, kMinCameraDistance, kMaxCameraDistance);
  }

  playerController->update(cameraAngleX,
                           GameSettings::Character::MOVEMENT_SPEED);
}

void Application::update(float deltaTime) {
  if (gameManager) gameManager->update(deltaTime);
  if (world) world->update(deltaTime);
}

void Application::render() {
  updateCameraTarget();
  updateCameraPosition();

  if (shaderSystem) shaderSystem->updateUniforms(camera);

  renderer.beginFrame();
  renderer.clearBackground(RAYWHITE);
  renderer.begin3D(camera);

  if (world) world->draw();
  if (gameManager) gameManager->render(camera);

  renderer.end3D();
  renderer.endFrame();
}

void Application::updateCameraTarget() {
  if (!playerCharacter) return;
  const Vector3 playerPos = playerCharacter->getFeetPosition();
  camera.target = {playerPos.x,
                   playerPos.y + GameSettings::Camera::TARGET_Y_OFFSET,
                   playerPos.z};
}

void Application::updateCameraPosition() {
  if (!playerCharacter) return;

  const Vector3 playerPos = playerCharacter->getFeetPosition();
  const float angleXRad = cameraAngleX * DEG2RAD;
  const float angleYRad = cameraAngleY * DEG2RAD;

  camera.position = {
      playerPos.x + cameraDistance * cosf(angleYRad) * sinf(angleXRad),
      playerPos.y + GameSettings::Camera::TARGET_Y_OFFSET +
          cameraDistance * sinf(angleYRad),
      playerPos.z + cameraDistance * cosf(angleYRad) * cosf(angleXRad)};
}
