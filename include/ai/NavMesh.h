#pragma once

#include <unordered_set>
#include <vector>

#include "raylib.h"

// Forward declaration
class GameObject;

// Navigation node for A* pathfinding
struct NavNode {
  Vector3 position;
  std::vector<int> connections;  // Indices of connected nodes
  bool walkable = true;
  float gCost = 0.0f;  // Cost from start
  float hCost = 0.0f;  // Cost to end (heuristic)
  float fCost = 0.0f;  // Total cost (g + h)
  int parent = -1;     // Parent node index for path reconstruction

  NavNode(Vector3 pos) : position(pos) {}
};

// Object registration for automatic pathfinding updates
struct RegisteredObject {
  GameObject* object;
  Vector3 lastPosition;
  Vector3 lastSize;
  bool wasBlocking;

  RegisteredObject(GameObject* obj)
      : object(obj),
        lastPosition({0, 0, 0}),
        lastSize({0, 0, 0}),
        wasBlocking(false) {}
};

// Navigation mesh for pathfinding
class NavMesh {
 private:
  std::vector<NavNode> nodes;
  float nodeSpacing;
  Vector3 minBounds;
  Vector3 maxBounds;
  float groundLevel;  // Y level of the navmesh ground
  float npcHeight;    // Height of NPCs for blocking calculations

  std::unordered_set<int> blockedNodeLookup;
  std::vector<int> blockedNodeCache;

  // Global object registration for automatic updates
  std::vector<RegisteredObject> registeredObjects;
  std::unordered_set<GameObject*> objectSet;  // For fast lookup

  void setNodeWalkable(int nodeIndex, bool walkable);

 public:
  NavMesh(Vector3 minBounds, Vector3 maxBounds, float spacing = 1.0f,
          float groundY = 0.0f);

  // Setup
  void generateNavMesh();
  void rebuildConnections();

  // Professional object management system
  void registerObject(GameObject* object);
  void unregisterObject(GameObject* object);
  void updateAllRegisteredObjects();
  void updateObjectBlocking(GameObject* object);

  // Advanced blocking calculation
  bool shouldObjectBlockPath(GameObject* object) const;
  float calculateBlockingHeight(GameObject* object) const;
  void markObjectNodes(GameObject* object, bool walkable);

  // Legacy obstacle management system (kept for compatibility)
  void addObstacle(Vector3 position, Vector3 size,
                   const std::string& type = "generic");
  void addShelfObstacle(Vector3 shelfPos, Vector3 shelfSize);
  void addWallObstacle(Vector3 wallPos, Vector3 wallSize);
  void removeObstacle(Vector3 position, Vector3 size);
  void markNodesInArea(Vector3 center, Vector3 size, bool walkable,
                       float expansionFactor = 0.5f);
  bool isPositionBlocked(Vector3 position) const;

  // Pathfinding
  std::vector<Vector3> findPath(Vector3 start, Vector3 end);
  std::vector<Vector3> findAlternativePath(Vector3 start, Vector3 end,
                                           Vector3 blockedArea);
  int findNearestNode(Vector3 position) const;
  bool isWalkable(Vector3 position) const;

  // A* algorithm implementation
  std::vector<int> aStar(int startNode, int endNode);

  // Utility
  float calculateDistance(int nodeA, int nodeB) const;
  float calculateHeuristic(int nodeA, int nodeB) const;
  float calculateImprovedHeuristic(int nodeA, int nodeB) const;
  float calculateMovementPenalty(int currentNode, int neighborNode,
                                 int targetNode) const;
  std::vector<int> smoothPath(const std::vector<int>& originalPath) const;
  int countNearbyObstacles(Vector3 position) const;
  void connectNodes();
  bool hasLineOfSight(Vector3 from, Vector3 to) const;

  // Configuration
  void setNPCHeight(float height) { npcHeight = height; }
  float getNPCHeight() const { return npcHeight; }
  void setGroundLevel(float y) { groundLevel = y; }
  float getGroundLevel() const { return groundLevel; }

  // Debug
  void debugDraw() const;
  void debugDrawRegisteredObjects() const;
  const std::vector<NavNode>& getNodes() const { return nodes; }
  size_t getRegisteredObjectCount() const { return registeredObjects.size(); }
};
