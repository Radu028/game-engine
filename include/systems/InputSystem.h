#ifndef INPUTSYSTEM_H
#define INPUTSYSTEM_H

#include "raylib.h"

class InputSystem {
 public:
  // Mouse camera controls (Roblox-style)
  static bool isRightMousePressed();
  static bool isRightMouseDown();
  static bool isRightMouseReleased();
  static Vector2 getMouseDelta();
  static void updateMouseCamera();
  static void enableMouseCamera();
  static void disableMouseCamera();

 private:
  static bool mouseCameraEnabled;
  static Vector2 lastMousePosition;
};

#endif