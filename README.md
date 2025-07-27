# 3D Game Engine

A comprehensive 3D game engine written in C++ showcasing advanced Object-Oriented Programming techniques, modern software architecture patterns, and real-time systems integration. Built with raylib graphics library and Bullet Physics engine.

## Overview

This project demonstrates a complete 3D interactive environment featuring:

- **Character-based gameplay** with realistic physics simulation and responsive controls
- **Intelligent NPC systems** utilizing advanced pathfinding algorithms and behavioral state machines
- **Dynamic economic simulation** with interactive trading mechanics and shopping behaviors
- **Real-time physics integration** supporting complex object interactions and collision detection
- **Modular architecture** enabling extensible gameplay systems and component composition

## Technical Architecture

### Object-Oriented Design Patterns

- **Inheritance and Polymorphism**: Robust class hierarchy with `GameObject` as abstract base class
- **Memory Management**: Smart pointer architecture eliminating manual memory management
- **Template-Based Programming**: Generic object management and type-safe operations
- **Exception Safety**: Comprehensive exception hierarchy for robust error handling
- **Design Patterns**: Implementation of Singleton, State, Observer patterns

### AI Systems

- **A* Pathfinding**: Optimized path calculation with Manhattan distance heuristic
- **Navigation Mesh**: Automated generation from world geometry with obstacle detection
- **Behavioral State Machine**: Complex NPC behaviors with state transitions
- **Dynamic Path Recalculation**: Real-time environment awareness

### Core Systems

- **Physics Integration**: Bullet Physics engine with custom character controller
- **Rendering**: raylib 3D graphics with custom shader pipeline
- **Component Architecture**: Modular system design with separation of concerns
- **Economic Simulation**: Trading system with dynamic pricing algorithms

## Screenshots

*Coming soon*

## System Requirements

- C++17 compatible compiler (GCC 7+, Clang 5+, or MSVC 2017+)
- CMake 3.16+ build system
- raylib graphics library
- Bullet Physics simulation library

## Build Instructions

### Linux/macOS
```bash
git clone https://github.com/Radu028/game.git
cd game
mkdir build && cd build
cmake ..
make -j$(nproc)
./GameProject
```

### Windows
```cmd
git clone https://github.com/Radu028/game.git
cd game
mkdir build && cd build
cmake .. -G "Visual Studio 16 2019"
cmake --build . --config Release
Release\GameProject.exe
```

## Controls

- **Movement**: WASD keys for directional character control
- **Camera**: Mouse control for 3D perspective adjustment
- **Debug**: F1 for debug mode, F2 for dynamic sun toggle
- **Reset**: R key to reset the game

## License

MIT License

## Author

Radu Popa

