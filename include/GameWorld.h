#ifndef GAMEWORLD_H
#define GAMEWORLD_H

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "ai/NavMesh.h"

class PhysicsSystem;
class GameObject;
class btDiscreteDynamicsWorld;

class GameWorld {
 private:
  static GameWorld* instance;
  static size_t totalObjectsCreated;
  static size_t totalObjectsDestroyed;
  static std::unordered_map<std::string, size_t> objectTypeCount;
  static std::string creationTimestamp;

  std::vector<std::shared_ptr<GameObject>> objects;
  std::unique_ptr<PhysicsSystem> physicsSystem;
  std::shared_ptr<NavMesh> navigationMesh;
  std::string worldName;

  explicit GameWorld(const std::string& name = "DefaultWorld");

  GameWorld(const GameWorld&) = delete;
  GameWorld& operator=(const GameWorld&) = delete;
  ~GameWorld();

 public:
  static GameWorld* getInstance();
  static void destroyInstance();

  static size_t getTotalObjectsCreated() { return totalObjectsCreated; }
  static size_t getTotalObjectsDestroyed() { return totalObjectsDestroyed; }
  static size_t getActiveObjectCount() {
    return totalObjectsCreated - totalObjectsDestroyed;
  }
  static size_t getTotalObjectCount() { return totalObjectsCreated; }
  static const std::unordered_map<std::string, size_t>& getObjectTypeCount() {
    return objectTypeCount;
  }
  static const std::string& getCreationTimestamp() { return creationTimestamp; }
  static void resetStatistics();

  template <typename T>
  std::vector<std::shared_ptr<T>> findObjectsOfType() const {
    static_assert(std::is_base_of_v<GameObject, T>,
                  "T must be derived from GameObject");

    std::vector<std::shared_ptr<T>> result;
    for (const auto& obj : objects) {
      if (obj) {
        if (auto castedObj = std::dynamic_pointer_cast<T>(obj)) {
          result.push_back(castedObj);
        }
      }
    }
    return result;
  }

  template <typename T>
  std::shared_ptr<T> findFirstObjectOfType() const {
    static_assert(std::is_base_of_v<GameObject, T>,
                  "T must be derived from GameObject");

    for (const auto& obj : objects) {
      if (obj) {
        if (auto castedObj = std::dynamic_pointer_cast<T>(obj)) {
          return castedObj;
        }
      }
    }
    return nullptr;
  }

  void addObject(std::shared_ptr<GameObject> object);
  void removeObject(std::shared_ptr<GameObject> object);
  void removeObjectsOfType(const std::string& typeName);
  void clearAllObjects();

  // Obstacle management
  void addObjectAsObstacle(std::shared_ptr<GameObject> object,
                           const std::string& type = "generic");
  void removeObjectObstacle(std::shared_ptr<GameObject> object);
  void addObstacleAt(Vector3 position, Vector3 size,
                     const std::string& type = "generic");
  void addObjectAsObstacleDeferred(std::shared_ptr<GameObject> object,
                                   const std::string& type = "generic");
  void finalizeObstacles();  // Call this to rebuild connections after adding
                             // multiple obstacles

  // World operations
  void update(float deltaTime);
  void draw() const;

  // Physics access
  btDiscreteDynamicsWorld* getBulletWorld() const;
  btDiscreteDynamicsWorld* getDynamicsWorld() const { return getBulletWorld(); }

  // Navigation
  void initializeNavMesh();
  std::shared_ptr<NavMesh> getNavMesh() const { return navigationMesh; }

  const std::vector<std::shared_ptr<GameObject>>& getObjects() const {
    return objects;
  }
  size_t getObjectCount() const { return objects.size(); }
  const std::string& getWorldName() const { return worldName; }

  void setWorldName(const std::string& name) { worldName = name; }

 private:
  static void incrementObjectCount(const std::string& typeName);
  static void decrementObjectCount(const std::string& typeName);
  std::string getObjectTypeName(const GameObject* obj) const;
};

#endif