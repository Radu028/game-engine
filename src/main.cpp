#include "GameObject.h"
#include "GameWorld.h"
#include "Physics.h"
#include "Player.h"
#include "raylib.h"

int main() {
  InitWindow(1280, 720, "Joc 3D");
  SetTargetFPS(120);

  DisableCursor();

  Camera3D camera = {0};
  camera.position = (Vector3){0.0f, 5.0f, 15.0f};
  camera.target = (Vector3){0.0f, 1.0f, 0.0f};
  camera.up = (Vector3){0.0f, 1.0f, 0.0f};
  camera.fovy = 45.0f;
  camera.projection = CAMERA_PERSPECTIVE;

  Player player1((Vector3){0.0f, 2.0f, 0.0f});
  GameWorld* world = GameWorld::getInstance(&player1);
  player1.setWorld(world);

  world->addObject(std::make_shared<CubeObject>((Vector3){2.0f, 0.5f, 0.0f},
                                                1.0f, 1.0f, 1.0f, PINK, true));

  // Add ground as a CubeObject with texture and collision
  world->addObject(std::make_shared<CubeObject>(
      (Vector3){0.0f, -0.05f, 0.0f}, 50.0f, 0.1f, 50.0f, GREEN, true,
      "../resources/forrest_ground_01_diff_4k.jpg"));

  while (!WindowShouldClose()) {
    player1.handleInput(0.1f);

    world->update(GetFrameTime());

    BeginDrawing();
    ClearBackground(RAYWHITE);

    BeginMode3D(camera);
    player1.draw();
    world->draw();
    EndMode3D();

    EndDrawing();
  }

  CloseWindow();
  return 0;
}