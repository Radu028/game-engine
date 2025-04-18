cmake_minimum_required(VERSION 3.10)
project(GameProject)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find_package(raylib REQUIRED)
# find_package(Bullet, REQUIRED)

add_subdirectory(external/raylib)
add_subdirectory(external/bullet)

include_directories(
  include
  external/bullet/src
)

file(GLOB SOURCES "src/*.cpp")
add_executable(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} 
  PRIVATE
    raylib
    BulletDynamics
    BulletCollision
    LinearMath
)