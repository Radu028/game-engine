#pragma once

#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include "raylib.h"

class PhysicsEngine;
class GameObject;
class NavMesh;
class btDiscreteDynamicsWorld;

class GameWorld {
 private:
  static GameWorld* instance;
  std::vector<std::shared_ptr<GameObject>> objects;
  std::unique_ptr<PhysicsEngine> physicsSystem;
  std::shared_ptr<NavMesh> navigationMesh;
  std::string worldName;

  explicit GameWorld(const std::string& name = "DefaultWorld");

  GameWorld(const GameWorld&) = delete;
  GameWorld& operator=(const GameWorld&) = delete;
  ~GameWorld();

 public:
  static GameWorld* getInstance();
  static void destroyInstance();

  template <typename T>
  std::vector<std::shared_ptr<T>> findObjectsOfType() const {
    static_assert(std::is_base_of_v<GameObject, T>,
                  "T must be derived from GameObject");

    std::vector<std::shared_ptr<T>> result;
    for (const auto& obj : objects) {
      if (!obj) continue;
      if (auto castedObj = std::dynamic_pointer_cast<T>(obj)) {
        result.push_back(castedObj);
      }
    }
    return result;
  }

  template <typename T>
  std::shared_ptr<T> findFirstObjectOfType() const {
    static_assert(std::is_base_of_v<GameObject, T>,
                  "T must be derived from GameObject");

    for (const auto& obj : objects) {
      if (!obj) continue;
      if (auto castedObj = std::dynamic_pointer_cast<T>(obj)) {
        return castedObj;
      }
    }
    return nullptr;
  }

  void addObject(std::shared_ptr<GameObject> object);
  void removeObject(std::shared_ptr<GameObject> object);

  void addObjectAsObstacle(std::shared_ptr<GameObject> object,
                           const std::string& type = "generic");
  void removeObjectObstacle(std::shared_ptr<GameObject> object);
  void addObstacleAt(Vector3 position, Vector3 size,
                     const std::string& type = "generic");
  void addObjectAsObstacleDeferred(std::shared_ptr<GameObject> object,
                                   const std::string& type = "generic");
  void finalizeObstacles();

  void update(float deltaTime);
  void draw() const;

  btDiscreteDynamicsWorld* getBulletWorld() const;
  btDiscreteDynamicsWorld* getDynamicsWorld() const { return getBulletWorld(); }

  void initializeNavMesh();
  std::shared_ptr<NavMesh> getNavMesh() const { return navigationMesh; }

  const std::vector<std::shared_ptr<GameObject>>& getObjects() const {
    return objects;
  }
  size_t getObjectCount() const { return objects.size(); }
  const std::string& getWorldName() const { return worldName; }

  void setWorldName(const std::string& name) { worldName = name; }
  void setPhysicsEngine(std::unique_ptr<PhysicsEngine> engine);
};