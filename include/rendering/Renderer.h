#pragma once

#include "raylib.h"

class Renderer {
 public:
  Renderer();
  ~Renderer();

  void initialize(int width, int height, const char* title);
  void setTargetFPS(int fps) const;
  bool shouldClose() const;
  float frameTime() const;

  void beginFrame() const;
  void endFrame() const;
  void clearBackground(Color color) const;
  void begin3D(const Camera3D& camera) const;
  void end3D() const;

 private:
  bool initialized;
};


