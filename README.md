# 3D Game Engine

A C++23 3D engine and interactive simulation built with raylib and Bullet Physics.

The project focuses on the systems behind a small game world: object ownership, rendering, physics, character control, pathfinding, NPC behavior, and shop interactions.

## Highlights

- Dynamic navigation mesh generated from walkable world geometry.
- A* pathfinding with obstacle-aware route updates and path smoothing.
- State-driven NPC behavior for roaming, shopping, waiting, and leaving.
- Bullet-based collision and rigid-body integration behind a small physics abstraction.
- Humanoid character controller with movement, jumping, and body-part composition.
- raylib rendering with custom lighting shaders.
- Shared object management and storage systems using modern C++ ownership.

## Architecture

```text
src/
├── ai/             navmesh, pathfinding, NPCs, and behavior states
├── app/            application lifecycle
├── controllers/    player input and character control
├── entities/       humanoid entities
├── game/           high-level game state
├── objects/        world object hierarchy
├── physics/        Bullet integration
├── rendering/      raylib rendering
├── shop/           shelves, fruit, and shopping behavior
└── systems/        input and shader systems
```

The main abstractions are deliberately separated: AI code queries the navigation representation, game objects own world behavior, and the physics layer hides Bullet-specific setup from the rest of the engine.

## Build

Requirements:

- a compiler with C++23 support;
- CMake 3.10 or newer;
- raylib;
- Bullet Physics.

```bash
git clone https://github.com/Radu028/game-engine.git
cd game-engine
cmake -S . -B build
cmake --build build -j
./build/GameProject
```

Package-manager paths for raylib and Bullet may need to be exposed through `CMAKE_PREFIX_PATH` on macOS.

## Controls

- `WASD` or arrow keys — move
- mouse — control the camera
- `Space` — jump

## License

[MIT](LICENSE)
