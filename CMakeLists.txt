cmake_minimum_required(VERSION 3.25)
project(GameProject LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(cmake/CPM.cmake)

# Configure Raylib build options before fetching
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_EXTRAS OFF CACHE BOOL "" FORCE)
set(BUILD_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(INSTALL_LIBS OFF CACHE BOOL "" FORCE)

CPMAddPackage(
  NAME        raylib
  GIT_REPOSITORY https://github.com/raysan5/raylib.git
  GIT_TAG        5.5
)

# Find the Bullet library and get include dirs and library list vars
find_package(Bullet 3.25 REQUIRED)

# Removed global include_directories to use target-based includes only

file(GLOB SOURCES "src/*.cpp")
add_executable(${PROJECT_NAME} ${SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_include_directories(${PROJECT_NAME} PRIVATE ${BULLET_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE
  raylib
  ${BULLET_LIBRARIES}
)

if(APPLE)
  target_link_libraries(${PROJECT_NAME} PRIVATE
    "-framework CoreVideo" "-framework IOKit"
    "-framework Cocoa" "-framework OpenGL")
endif()