cmake_minimum_required(VERSION 3.16)
project(GameProject)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Verify C++23 compiler support
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++23" COMPILER_SUPPORTS_CXX23)
if(NOT COMPILER_SUPPORTS_CXX23)
    message(WARNING "The compiler may not fully support C++23")
endif()

# Helpful for IDE tooling (clangd/Cursor) and debugging build issues.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE BOOL "Export compile commands" FORCE)

find_package(raylib REQUIRED)
find_package(Bullet REQUIRED)

file(GLOB SRC_ROOT_FILES CONFIGURE_DEPENDS "src/*.cpp")
file(GLOB_RECURSE SRC_SUBDIR_FILES CONFIGURE_DEPENDS "src/*/*.cpp")

set(ALL_GAME_SOURCES ${SRC_ROOT_FILES} ${SRC_SUBDIR_FILES})

add_executable(${PROJECT_NAME} ${ALL_GAME_SOURCES})

# Use modern CMake target-based include directories
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${BULLET_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PRIVATE raylib ${BULLET_LIBRARIES})
