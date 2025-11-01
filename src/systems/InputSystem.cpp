#include "systems/InputSystem.h"

bool InputSystem::mouseCameraEnabled = false;
Vector2 InputSystem::lastMousePosition = {0.0f, 0.0f};

bool InputSystem::isRightMousePressed() {
  return IsMouseButtonPressed(MOUSE_BUTTON_RIGHT);
}

bool InputSystem::isRightMouseDown() {
  return IsMouseButtonDown(MOUSE_BUTTON_RIGHT);
}

bool InputSystem::isRightMouseReleased() {
  return IsMouseButtonReleased(MOUSE_BUTTON_RIGHT);
}

Vector2 InputSystem::getMouseDelta() {
  Vector2 currentMousePosition = GetMousePosition();
  Vector2 delta = {0.0f, 0.0f};

  if (mouseCameraEnabled) {
    delta.x = currentMousePosition.x - lastMousePosition.x;
    delta.y = currentMousePosition.y - lastMousePosition.y;
    lastMousePosition = currentMousePosition;
  }

  return delta;
}

void InputSystem::updateMouseCamera() {
  if (isRightMousePressed()) {
    enableMouseCamera();
  }

  if (isRightMouseReleased()) {
    disableMouseCamera();
  }

  if (!mouseCameraEnabled) {
    lastMousePosition = GetMousePosition();
  }
}

void InputSystem::enableMouseCamera() {
  mouseCameraEnabled = true;
  lastMousePosition = GetMousePosition();
}

void InputSystem::disableMouseCamera() { mouseCameraEnabled = false; }