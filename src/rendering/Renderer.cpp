#include "rendering/Renderer.h"

Renderer::Renderer() : initialized(false) {}

Renderer::~Renderer() {
  if (initialized) CloseWindow();
}

void Renderer::initialize(int width, int height, const char* title) {
  if (initialized) return;
  InitWindow(width, height, title);
  initialized = true;
}

void Renderer::setTargetFPS(int fps) const { SetTargetFPS(fps); }

bool Renderer::shouldClose() const { return WindowShouldClose(); }

float Renderer::frameTime() const { return GetFrameTime(); }

void Renderer::beginFrame() const { BeginDrawing(); }

void Renderer::endFrame() const { EndDrawing(); }

void Renderer::clearBackground(Color color) const { ClearBackground(color); }

void Renderer::begin3D(const Camera3D& camera) const { BeginMode3D(camera); }

void Renderer::end3D() const { EndMode3D(); }


