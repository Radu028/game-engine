#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <type_traits>
#include <vector>

#include "exceptions/GameExceptions.h"
#include "raylib.h"

// Template class for managing collections of game objects with type safety
template <typename T>
class GameObjectManager {
  static_assert(std::is_base_of_v<class GameObject, T>,
                "T must be derived from GameObject");

 private:
  std::vector<std::shared_ptr<T>> objects;
  std::string managerName;
  size_t maxCapacity;

  static size_t totalManagersCreated;

 public:
  explicit GameObjectManager(const std::string& name, size_t capacity = 100)
      : managerName(name), maxCapacity(capacity) {
    objects.reserve(capacity);
    ++totalManagersCreated;
  }

  ~GameObjectManager() = default;

  GameObjectManager(const GameObjectManager& other)
      : managerName(other.managerName + "_copy"),
        maxCapacity(other.maxCapacity) {
    objects.reserve(other.objects.size());
    for (const auto& obj : other.objects) {
      if (obj) {
        // Use clone method for deep copy - clone returns unique_ptr
        auto cloned_unique = obj->clone();
        // Convert unique_ptr to shared_ptr
        auto cloned =
            std::shared_ptr<T>(dynamic_cast<T*>(cloned_unique.release()));
        if (cloned) {
          objects.push_back(cloned);
        }
      }
    }
    ++totalManagersCreated;
  }

  GameObjectManager& operator=(GameObjectManager other) {
    swap(*this, other);
    return *this;
  }

  GameObjectManager(GameObjectManager&& other) noexcept
      : objects(std::move(other.objects)),
        managerName(std::move(other.managerName)),
        maxCapacity(other.maxCapacity) {
    other.maxCapacity = 0;
  }

  friend void swap(GameObjectManager& first,
                   GameObjectManager& second) noexcept {
    using std::swap;
    swap(first.objects, second.objects);
    swap(first.managerName, second.managerName);
    swap(first.maxCapacity, second.maxCapacity);
  }

  void addObject(std::shared_ptr<T> object) {
    if (!object) {
      throw GameException("Cannot add null object to " + managerName);
    }

    if (objects.size() >= maxCapacity) {
      throw GameException(
          "Manager " + managerName +
          " has reached maximum capacity: " + std::to_string(maxCapacity));
    }

    objects.push_back(object);
  }

  bool removeObject(std::shared_ptr<T> object) {
    auto it = std::find(objects.begin(), objects.end(), object);
    if (it != objects.end()) {
      objects.erase(it);
      return true;
    }
    return false;
  }

  template <typename Predicate>
  std::vector<std::shared_ptr<T>> findObjects(Predicate pred) const {
    std::vector<std::shared_ptr<T>> result;
    std::copy_if(objects.begin(), objects.end(), std::back_inserter(result),
                 pred);
    return result;
  }

  template <typename Function>
  void forEachObject(Function func) {
    std::for_each(objects.begin(), objects.end(), func);
  }

  std::shared_ptr<T> getObject(size_t index) const {
    if (index >= objects.size()) {
      throw GameException("Index " + std::to_string(index) +
                          " out of bounds for manager " + managerName);
    }
    return objects[index];
  }

  template <typename DerivedT>
  std::vector<std::shared_ptr<DerivedT>> getObjectsOfType() const {
    static_assert(std::is_base_of_v<T, DerivedT>,
                  "DerivedT must be derived from T");

    std::vector<std::shared_ptr<DerivedT>> result;
    for (const auto& obj : objects) {
      if (auto derived = std::dynamic_pointer_cast<DerivedT>(obj)) {
        result.push_back(derived);
      }
    }
    return result;
  }

  size_t size() const { return objects.size(); }
  size_t capacity() const { return maxCapacity; }
  bool empty() const { return objects.empty(); }
  const std::string& getName() const { return managerName; }

  static size_t getTotalManagersCreated() { return totalManagersCreated; }

  typename std::vector<std::shared_ptr<T>>::iterator begin() {
    return objects.begin();
  }
  typename std::vector<std::shared_ptr<T>>::iterator end() {
    return objects.end();
  }
  typename std::vector<std::shared_ptr<T>>::const_iterator begin() const {
    return objects.begin();
  }
  typename std::vector<std::shared_ptr<T>>::const_iterator end() const {
    return objects.end();
  }
  typename std::vector<std::shared_ptr<T>>::const_iterator cbegin() const {
    return objects.cbegin();
  }
  typename std::vector<std::shared_ptr<T>>::const_iterator cend() const {
    return objects.cend();
  }
};

// Initialize static member
template <typename T>
size_t GameObjectManager<T>::totalManagersCreated = 0;

// Template function for distance calculation between any two objects with
// position
template <typename T1, typename T2>
float calculateDistance(const T1& obj1, const T2& obj2) {
  static_assert(std::is_same_v<decltype(obj1.getPosition()), Vector3>,
                "T1 must have getPosition() method returning Vector3");
  static_assert(std::is_same_v<decltype(obj2.getPosition()), Vector3>,
                "T2 must have getPosition() method returning Vector3");

  Vector3 pos1 = obj1.getPosition();
  Vector3 pos2 = obj2.getPosition();

  float dx = pos1.x - pos2.x;
  float dy = pos1.y - pos2.y;
  float dz = pos1.z - pos2.z;

  return sqrtf(dx * dx + dy * dy + dz * dz);
}

// Template function for finding nearest object
template <typename T, typename Container>
std::shared_ptr<T> findNearestObject(const Vector3& position,
                                     const Container& objects) {
  if (objects.empty()) {
    return nullptr;
  }

  std::shared_ptr<T> nearest = nullptr;
  float minDistance = std::numeric_limits<float>::max();

  for (const auto& obj : objects) {
    if (!obj) continue;

    Vector3 objPos = obj->getPosition();
    float dx = position.x - objPos.x;
    float dy = position.y - objPos.y;
    float dz = position.z - objPos.z;
    float distance =
        dx * dx + dy * dy + dz * dz;  // Using squared distance for performance

    if (distance < minDistance) {
      minDistance = distance;
      nearest = obj;
    }
  }

  return nearest;
}

// Template function for spatial partitioning/grouping
template <typename T>
std::vector<std::vector<std::shared_ptr<T>>> groupObjectsByDistance(
    const std::vector<std::shared_ptr<T>>& objects, float maxDistance) {
  std::vector<std::vector<std::shared_ptr<T>>> groups;
  std::vector<bool> processed(objects.size(), false);

  for (size_t i = 0; i < objects.size(); ++i) {
    if (processed[i] || !objects[i]) continue;

    std::vector<std::shared_ptr<T>> group;
    group.push_back(objects[i]);
    processed[i] = true;

    Vector3 centerPos = objects[i]->getPosition();

    // Find all objects within maxDistance of this object
    for (size_t j = i + 1; j < objects.size(); ++j) {
      if (processed[j] || !objects[j]) continue;

      Vector3 objPos = objects[j]->getPosition();
      float dx = centerPos.x - objPos.x;
      float dy = centerPos.y - objPos.y;
      float dz = centerPos.z - objPos.z;
      float distance = sqrtf(dx * dx + dy * dy + dz * dz);

      if (distance <= maxDistance) {
        group.push_back(objects[j]);
        processed[j] = true;
      }
    }

    groups.push_back(std::move(group));
  }

  return groups;
}
