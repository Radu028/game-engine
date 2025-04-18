cmake_minimum_required(VERSION 3.25)
project(GameProject LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

FetchContent_Declare(
  raylib
  GIT_REPOSITORY https://github.com/raysan5/raylib.git
  GIT_TAG        5.5
)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(raylib)

FetchContent_Declare(
  reactphysics3d
  GIT_REPOSITORY https://github.com/DanielChappuis/ReactPhysics3D.git
  GIT_TAG        0.10.2
)
set(RP3D_COMPILE_TESTBED OFF CACHE BOOL "" FORCE)
set(RP3D_COMPILE_TESTS   OFF CACHE BOOL "" FORCE)
set(RP3D_PROFILING_ENABLED OFF CACHE BOOL "" FORCE)
set(RP3D_DOUBLE_PRECISION_ENABLED OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(reactphysics3d)

include_directories(include)

file(GLOB SOURCES "src/*.cpp")
add_executable(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} PRIVATE
  raylib
  ReactPhysics3D::reactphysics3d
)

if(APPLE)
  target_link_libraries(${PROJECT_NAME} PRIVATE
      "-framework CoreVideo" "-framework IOKit"
      "-framework Cocoa" "-framework OpenGL")
endif()