cmake_minimum_required(VERSION 3.16)
project(GameProject LANGUAGES CXX)

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

option(ENABLE_DEV_WARNINGS "Enable extra compiler warnings" ON)
option(ENABLE_SANITIZERS "Enable AddressSanitizer + UndefinedBehaviorSanitizer" OFF)

find_package(raylib REQUIRED)
find_package(Bullet REQUIRED)

file(GLOB SRC_ROOT_FILES CONFIGURE_DEPENDS "src/*.cpp")
file(GLOB_RECURSE SRC_SUBDIR_FILES CONFIGURE_DEPENDS "src/*/*.cpp")
set(ALL_GAME_SOURCES ${SRC_ROOT_FILES} ${SRC_SUBDIR_FILES})

set(GAME_CORE_SOURCES ${ALL_GAME_SOURCES})
list(REMOVE_ITEM GAME_CORE_SOURCES "${CMAKE_SOURCE_DIR}/src/main.cpp")

add_library(GameProjectCore STATIC ${GAME_CORE_SOURCES})
target_include_directories(GameProjectCore PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_include_directories(GameProjectCore SYSTEM PUBLIC ${BULLET_INCLUDE_DIRS})
target_link_libraries(GameProjectCore PUBLIC raylib ${BULLET_LIBRARIES})

if(ENABLE_DEV_WARNINGS)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        target_compile_options(GameProjectCore PRIVATE -Wall -Wextra)
    endif()
endif()

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE GameProjectCore)

if(ENABLE_SANITIZERS AND CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(GameProjectCore PRIVATE
        -fsanitize=address,undefined
        -fno-omit-frame-pointer
    )
    target_link_options(${PROJECT_NAME} PRIVATE -fsanitize=address,undefined)
endif()

include(CTest)
if(BUILD_TESTING)
    add_executable(SettingsTests tests/SettingsTests.cpp)
    target_include_directories(SettingsTests PRIVATE ${CMAKE_SOURCE_DIR}/tests)
    target_link_libraries(SettingsTests PRIVATE GameProjectCore)

    add_executable(NavMeshTests tests/NavMeshTests.cpp)
    target_include_directories(NavMeshTests PRIVATE ${CMAKE_SOURCE_DIR}/tests)
    target_link_libraries(NavMeshTests PRIVATE GameProjectCore)

    add_executable(SmokeFlowTest tests/SmokeFlowTest.cpp)
    target_include_directories(SmokeFlowTest PRIVATE ${CMAKE_SOURCE_DIR}/tests)
    target_link_libraries(SmokeFlowTest PRIVATE GameProjectCore)

    if(ENABLE_SANITIZERS AND CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        foreach(TEST_TARGET SettingsTests NavMeshTests SmokeFlowTest)
            target_link_options(${TEST_TARGET} PRIVATE -fsanitize=address,undefined)
        endforeach()
    endif()

    add_test(NAME settings COMMAND SettingsTests)
    add_test(NAME navmesh COMMAND NavMeshTests)
    add_test(NAME smoke_flow COMMAND SmokeFlowTest --steps 12000 --obstacle-sets 12 --seed 1337)
    set_tests_properties(smoke_flow PROPERTIES TIMEOUT 240)
endif()
