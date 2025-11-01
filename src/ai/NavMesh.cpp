#include "ai/NavMesh.h"

#include <algorithm>
#include <cmath>
#include <queue>

#include "objects/GameObject.h"
#include "raymath.h"
#include "settings/Physics.h"

NavMesh::NavMesh(Vector3 minBounds, Vector3 maxBounds, float spacing,
                 float groundY)
    : minBounds(minBounds),
      maxBounds(maxBounds),
      nodeSpacing(spacing),
      groundLevel(groundY),
      npcHeight(GameSettings::Character::HEIGHT) {}

void NavMesh::generateNavMesh() {
  nodes.clear();

  for (float x = minBounds.x; x <= maxBounds.x; x += nodeSpacing) {
    for (float z = minBounds.z; z <= maxBounds.z; z += nodeSpacing) {
      Vector3 nodePos = {x, groundLevel + 0.5f, z};
      nodes.emplace_back(nodePos);
    }
  }

  connectNodes();
}

void NavMesh::connectNodes() {
  // Clear all existing connections first
  for (auto& node : nodes) {
    node.connections.clear();
  }

  // OPTIMIZED: O(n) grid-based connection instead of O(n²)
  // Calculate grid dimensions
  int gridWidth =
      static_cast<int>((maxBounds.x - minBounds.x) / nodeSpacing) + 1;
  int gridHeight =
      static_cast<int>((maxBounds.z - minBounds.z) / nodeSpacing) + 1;

  // Lambda to convert 2D grid coordinates to 1D node index
  auto getNodeIndex = [&](int x, int z) -> int {
    if (x < 0 || x >= gridWidth || z < 0 || z >= gridHeight) return -1;
    return z * gridWidth + x;
  };

  // Lambda to convert node index to grid coordinates
  auto getGridCoords = [&](int nodeIndex) -> std::pair<int, int> {
    int z = nodeIndex / gridWidth;
    int x = nodeIndex % gridWidth;
    return {x, z};
  };

  // Connect each node only to its immediate neighbors (8-connected)
  for (size_t i = 0; i < nodes.size(); ++i) {
    if (!nodes[i].walkable) continue;

    auto [gridX, gridZ] = getGridCoords(i);

    // Check 8 neighbors (4-directional + 4-diagonal)
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dz = -1; dz <= 1; ++dz) {
        if (dx == 0 && dz == 0) continue;  // Skip self

        int neighborIndex = getNodeIndex(gridX + dx, gridZ + dz);
        if (neighborIndex == -1 ||
            neighborIndex >= static_cast<int>(nodes.size()))
          continue;
        if (!nodes[neighborIndex].walkable) continue;

        // Add bidirectional connection
        nodes[i].connections.push_back(neighborIndex);
      }
    }
  }
}

// Removed addLimitedStrategicConnections() - too expensive O(n²) operation
// 8-connected grid is sufficient for good pathfinding

int NavMesh::countNearbyObstacles(Vector3 position) const {
  int count = 0;
  float checkRadius = nodeSpacing * 2.5f;

  for (const auto& node : nodes) {
    if (!node.walkable &&
        Vector3Distance(position, node.position) <= checkRadius) {
      count++;
    }
  }

  return count;
}

void NavMesh::addObstacle(Vector3 position, Vector3 size,
                          const std::string& type) {
  float expansionFactor = 0.5f;
  if (type == "shelf") {
    expansionFactor = 0.7f;
  } else if (type == "wall") {
    expansionFactor = 0.3f;
  }

  markNodesInArea(position, size, false, expansionFactor);

  // Don't rebuild connections here - do it once at the end in
  // initializeNavMesh()
}

void NavMesh::defineShopEntrance(Vector3 entrancePos, Vector3 entranceSize) {
  float entranceHalfWidth = entranceSize.x / 2.0f;
  float entranceDepth = entranceSize.z * 2.0f;

  float frontWallWidth = (20.0f - entranceSize.x) / 2.0f;
  float leftWallMaxX = entrancePos.x - entranceHalfWidth;
  float rightWallMinX = entrancePos.x + entranceHalfWidth;

  int safeNodesCount = 0;
  int edgeNodesCount = 0;
  int rejectedNodesCount = 0;

  for (auto& node : nodes) {
    Vector3 diff = Vector3Subtract(node.position, entrancePos);

    if (std::abs(diff.z) <= entranceDepth) {
      float safeMargin = nodeSpacing * 0.4f;

      if (diff.x >= (leftWallMaxX + safeMargin) &&
          diff.x <= (rightWallMinX - safeMargin)) {
        node.walkable = true;
        safeNodesCount++;
      } else if (std::abs(diff.x) <= entranceHalfWidth + safeMargin) {
        // Node is near entrance edge - do additional verification
        if (isNodeFullyAccessible(node.position, entrancePos, entranceSize) &&
            !isNodeTooCloseToWall(node.position, entrancePos, entranceSize)) {
          node.walkable = true;
          edgeNodesCount++;
        } else {
          rejectedNodesCount++;
        }
      }
    }
  }
}

void NavMesh::defineShopInterior(Vector3 shopPos, Vector3 shopSize) {
  // Mark interior of shop as walkable, but leave a small margin for walls
  float wallThickness = 1.0f;
  Vector3 interiorSize = {shopSize.x - wallThickness * 2.0f, shopSize.y,
                          shopSize.z - wallThickness * 2.0f};

  int interiorNodesCount = 0;
  for (auto& node : nodes) {
    Vector3 diff = Vector3Subtract(node.position, shopPos);
    if (std::abs(diff.x) <= interiorSize.x / 2.0f &&
        std::abs(diff.z) <= interiorSize.z / 2.0f) {
      node.walkable = true;
      interiorNodesCount++;
    }
  }
}

std::vector<Vector3> NavMesh::findPath(Vector3 start, Vector3 end) {
  int startNode = findNearestNode(start);
  int endNode = findNearestNode(end);

  if (startNode == -1 || endNode == -1) {
    return {};
  }

  if (!nodes[startNode].walkable || !nodes[endNode].walkable) {
    return {};
  }

  std::vector<int> pathNodes = aStar(startNode, endNode);

  if (pathNodes.empty()) {
    return {};
  }

  std::vector<Vector3> path;
  for (int nodeIndex : pathNodes) {
    if (nodeIndex >= 0 && nodeIndex < static_cast<int>(nodes.size())) {
      path.push_back(nodes[nodeIndex].position);
    }
  }

  return path;
}

int NavMesh::findNearestNode(Vector3 position) const {
  int nearestNode = -1;
  float minDistance = INFINITY;
  int bestConnectivityNode = -1;
  float bestConnectivityDistance = INFINITY;

  for (size_t i = 0; i < nodes.size(); ++i) {
    if (!nodes[i].walkable) continue;

    float distance = Vector3Distance(position, nodes[i].position);

    // Standard nearest node selection
    if (distance < minDistance) {
      minDistance = distance;
      nearestNode = static_cast<int>(i);
    }

    // Also consider nodes with better connectivity (more connections)
    // This can lead to better paths by starting/ending at well-connected nodes
    if (distance < nodeSpacing * 3.0f && nodes[i].connections.size() >= 4) {
      float connectivityScore =
          distance / (1.0f + nodes[i].connections.size() * 0.1f);
      if (connectivityScore < bestConnectivityDistance) {
        bestConnectivityDistance = connectivityScore;
        bestConnectivityNode = static_cast<int>(i);
      }
    }
  }

  // If we found a well-connected node that's reasonably close, prefer it
  if (bestConnectivityNode != -1 &&
      bestConnectivityDistance < minDistance * 1.5f &&
      minDistance > nodeSpacing * 0.8f) {
    return bestConnectivityNode;
  }

  return nearestNode;
}

bool NavMesh::isWalkable(Vector3 position) const {
  int nearestNode = -1;
  float minDistance = INFINITY;

  for (size_t i = 0; i < nodes.size(); ++i) {
    float distance = Vector3Distance(position, nodes[i].position);
    if (distance < minDistance) {
      minDistance = distance;
      nearestNode = static_cast<int>(i);
    }
  }

  if (nearestNode == -1 || minDistance > nodeSpacing * 1.5f) {
    return false;
  }

  return nodes[nearestNode].walkable;
}

std::vector<int> NavMesh::aStar(int startNode, int endNode) {
  // Reset all nodes
  for (auto& node : nodes) {
    node.gCost = INFINITY;
    node.hCost = 0.0f;
    node.fCost = INFINITY;
    node.parent = -1;
  }

  // Improved comparison function with tie-breaking by heuristic
  auto compare = [this](int a, int b) {
    if (std::abs(nodes[a].fCost - nodes[b].fCost) < 0.001f) {
      return nodes[a].hCost >
             nodes[b].hCost;  // Prefer lower heuristic for tie-breaking
    }
    return nodes[a].fCost > nodes[b].fCost;
  };

  std::priority_queue<int, std::vector<int>, decltype(compare)> openSet(
      compare);
  std::vector<bool> inOpenSet(nodes.size(), false);
  std::vector<bool> inClosedSet(nodes.size(), false);

  nodes[startNode].gCost = 0.0f;
  nodes[startNode].hCost = calculateHeuristic(startNode, endNode);
  nodes[startNode].fCost = nodes[startNode].hCost;

  openSet.push(startNode);
  inOpenSet[startNode] = true;

  // Early termination check for direct path
  if (hasLineOfSight(nodes[startNode].position, nodes[endNode].position)) {
    return {startNode, endNode};
  }

  while (!openSet.empty()) {
    int current = openSet.top();
    openSet.pop();
    inOpenSet[current] = false;

    if (current == endNode) {
      // Reconstruct and smooth path
      std::vector<int> path;
      int node = endNode;
      while (node != -1) {
        path.push_back(node);
        node = nodes[node].parent;
      }
      std::reverse(path.begin(), path.end());

      // Apply path smoothing
      return smoothPath(path);
    }

    inClosedSet[current] = true;

    for (int neighbor : nodes[current].connections) {
      if (!nodes[neighbor].walkable || inClosedSet[neighbor]) {
        continue;
      }

      // Improved cost calculation with movement penalties
      float baseCost = calculateDistance(current, neighbor);
      float movementPenalty =
          calculateMovementPenalty(current, neighbor, endNode);
      float tentativeGCost = nodes[current].gCost + baseCost + movementPenalty;

      if (tentativeGCost < nodes[neighbor].gCost) {
        nodes[neighbor].parent = current;
        nodes[neighbor].gCost = tentativeGCost;
        nodes[neighbor].hCost = calculateImprovedHeuristic(neighbor, endNode);
        nodes[neighbor].fCost = nodes[neighbor].gCost + nodes[neighbor].hCost;

        if (!inOpenSet[neighbor]) {
          openSet.push(neighbor);
          inOpenSet[neighbor] = true;
        }
      }
    }
  }

  return {};
}

float NavMesh::calculateDistance(int nodeA, int nodeB) const {
  return Vector3Distance(nodes[nodeA].position, nodes[nodeB].position);
}

float NavMesh::calculateHeuristic(int nodeA, int nodeB) const {
  Vector3 diff = Vector3Subtract(nodes[nodeA].position, nodes[nodeB].position);
  return std::abs(diff.x) + std::abs(diff.z);
}

float NavMesh::calculateImprovedHeuristic(int nodeA, int nodeB) const {
  Vector3 posA = nodes[nodeA].position;
  Vector3 posB = nodes[nodeB].position;
  Vector3 diff = Vector3Subtract(posB, posA);

  // Use Euclidean distance as base (more accurate than Manhattan)
  float euclideanDistance = Vector3Length(diff);

  // Add penalty for being near obstacles (encourages staying in open areas)
  float obstaclePenalty = 0.0f;
  int nearbyObstacles = 0;

  for (size_t i = 0; i < nodes.size(); ++i) {
    if (i == nodeA || nodes[i].walkable) continue;

    float distanceToObstacle = Vector3Distance(posA, nodes[i].position);
    if (distanceToObstacle < nodeSpacing * 2.0f) {
      nearbyObstacles++;
      obstaclePenalty += (nodeSpacing * 2.0f - distanceToObstacle) * 0.1f;
    }
  }

  // Slight preference for straight-line paths
  Vector3 direction = Vector3Normalize(diff);
  float straightnessPenalty = 0.0f;
  if (nodes[nodeA].parent != -1) {
    Vector3 prevDirection =
        Vector3Subtract(posA, nodes[nodes[nodeA].parent].position);
    if (Vector3Length(prevDirection) > 0.1f) {
      prevDirection = Vector3Normalize(prevDirection);
      float dot = Vector3DotProduct(direction, prevDirection);
      straightnessPenalty =
          (1.0f - dot) * 0.2f;  // Penalty for direction changes
    }
  }

  return euclideanDistance + obstaclePenalty + straightnessPenalty;
}

float NavMesh::calculateMovementPenalty(int currentNode, int neighborNode,
                                        int targetNode) const {
  Vector3 currentPos = nodes[currentNode].position;
  Vector3 neighborPos = nodes[neighborNode].position;
  Vector3 targetPos = nodes[targetNode].position;

  float penalty = 0.0f;

  // Penalty for diagonal movement (encourage axis-aligned movement when
  // possible)
  Vector3 moveDirection = Vector3Subtract(neighborPos, currentPos);
  if (std::abs(moveDirection.x) > 0.1f && std::abs(moveDirection.z) > 0.1f) {
    penalty += 0.1f;  // Small penalty for diagonal moves
  }

  // Penalty for moving away from target
  float currentDistanceToTarget = Vector3Distance(currentPos, targetPos);
  float neighborDistanceToTarget = Vector3Distance(neighborPos, targetPos);
  if (neighborDistanceToTarget > currentDistanceToTarget) {
    penalty += 0.05f;  // Very small penalty for moving away from target
  }

  // Bonus for moving toward areas with more connections (open areas)
  if (nodes[neighborNode].connections.size() > 6) {
    penalty -= 0.05f;  // Small bonus for open areas
  }

  return std::max(0.0f, penalty);
}

bool NavMesh::hasLineOfSight(Vector3 from, Vector3 to) const {
  Vector3 direction = Vector3Subtract(to, from);
  float distance = Vector3Length(direction);

  if (distance < 0.1f) return true;

  // For immediate neighbors (distance < 2.0f), assume line of sight is clear
  // to avoid expensive checking for grid-adjacent nodes
  if (distance <= nodeSpacing * 1.5f) return true;

  direction = Vector3Normalize(direction);

  // Reduced sampling for better performance - fewer checks but still effective
  int samples =
      std::max(2, static_cast<int>(distance / 1.0f));  // Larger step size

  for (int i = 1; i < samples; ++i) {
    float t = static_cast<float>(i) / static_cast<float>(samples);
    Vector3 samplePoint =
        Vector3Add(from, Vector3Scale(direction, distance * t));

    if (!isWalkable(samplePoint)) {
      return false;
    }
  }

  return true;
}

void NavMesh::debugDraw() const {
  for (size_t i = 0; i < nodes.size(); ++i) {
    const auto& node = nodes[i];
    Color nodeColor = node.walkable ? GREEN : RED;
    DrawCube(node.position, 0.2f, 0.2f, 0.2f, nodeColor);

    for (int connectionIndex : node.connections) {
      if (connectionIndex > static_cast<int>(i)) {
        DrawLine3D(node.position, nodes[connectionIndex].position, BLUE);
      }
    }
  }
}

void NavMesh::debugDrawEntranceNodes(Vector3 entrancePos,
                                     Vector3 entranceSize) const {
  float entranceHalfWidth = entranceSize.x / 2.0f;
  float entranceDepth = entranceSize.z * 2.0f;

  Vector3 entranceMin = {entrancePos.x - entranceHalfWidth, entrancePos.y,
                         entrancePos.z - entranceDepth};
  Vector3 entranceMax = {entrancePos.x + entranceHalfWidth, entrancePos.y,
                         entrancePos.z + entranceDepth};

  DrawCubeWires({entrancePos.x, entrancePos.y + 0.1f, entrancePos.z},
                entranceSize.x, 0.2f, entranceDepth, YELLOW);

  // Draw nodes in entrance area with different colors based on walkability
  for (const auto& node : nodes) {
    Vector3 diff = Vector3Subtract(node.position, entrancePos);

    // Check if node is in entrance area
    if (std::abs(diff.x) <= entranceHalfWidth + 1.0f &&
        std::abs(diff.z) <= entranceDepth) {
      Color nodeColor;
      if (node.walkable) {
        if (std::abs(diff.x) <= entranceHalfWidth - nodeSpacing * 0.4f) {
          nodeColor = GREEN;  // Safely inside entrance
        } else {
          nodeColor = ORANGE;  // Edge node marked as walkable
        }
      } else {
        nodeColor = RED;  // Not walkable
      }

      DrawSphere({node.position.x, node.position.y + 0.3f, node.position.z},
                 0.15f, nodeColor);
    }
  }
}

std::vector<Vector3> NavMesh::findAlternativePath(Vector3 start, Vector3 end,
                                                  Vector3 blockedArea) {
  int startNode = findNearestNode(start);
  int endNode = findNearestNode(end);

  if (startNode == -1 || endNode == -1) {
    return {};
  }

  // Strategy 1: Try to find a path that goes around the blocked area
  Vector3 startToEnd = Vector3Subtract(end, start);
  Vector3 startToBlocked = Vector3Subtract(blockedArea, start);

  // Determine which side to go around (left or right relative to start->end
  // direction)
  float crossProduct =
      startToEnd.x * startToBlocked.z - startToEnd.z * startToBlocked.x;
  bool goLeft = crossProduct > 0;

  // Create waypoints that go around the obstacle
  std::vector<Vector3> alternativeWaypoints;
  alternativeWaypoints.push_back(start);

  // Calculate perpendicular direction to go around obstacle
  Vector3 perpDirection;
  if (goLeft) {
    perpDirection = {-startToEnd.z, 0, startToEnd.x};  // Rotate 90 degrees left
  } else {
    perpDirection = {startToEnd.z, 0,
                     -startToEnd.x};  // Rotate 90 degrees right
  }
  perpDirection = Vector3Normalize(perpDirection);

  // Add intermediate waypoint to go around the obstacle
  float detourDistance =
      Vector3Distance(start, blockedArea) + nodeSpacing * 2.0f;
  Vector3 detourPoint =
      Vector3Add(blockedArea, Vector3Scale(perpDirection, detourDistance));
  detourPoint.y = 0.5f;  // Keep at ground level

  // Find nearest walkable node to detour point
  int detourNode = findNearestNode(detourPoint);
  if (detourNode != -1 && nodes[detourNode].walkable) {
    alternativeWaypoints.push_back(nodes[detourNode].position);
  }

  alternativeWaypoints.push_back(end);

  // Convert waypoints to actual path using navigation mesh
  std::vector<Vector3> fullAlternativePath;

  for (size_t i = 0; i < alternativeWaypoints.size() - 1; ++i) {
    std::vector<Vector3> segmentPath =
        findPath(alternativeWaypoints[i], alternativeWaypoints[i + 1]);

    if (segmentPath.empty()) {
      // If any segment fails, fall back to original blocked area avoidance
      break;
    }

    // Add segment to full path (avoid duplicating waypoints)
    if (fullAlternativePath.empty()) {
      fullAlternativePath.insert(fullAlternativePath.end(), segmentPath.begin(),
                                 segmentPath.end());
    } else {
      fullAlternativePath.insert(fullAlternativePath.end(),
                                 segmentPath.begin() + 1, segmentPath.end());
    }
  }

  // If strategic routing worked, return it
  if (!fullAlternativePath.empty() && fullAlternativePath.size() > 2) {
    return fullAlternativePath;
  }

  // Strategy 2: Fallback to original method with progressive blocking
  std::vector<float> blockRadii = {
      1.5f, 1.0f, 0.5f};  // Reduced radii for less aggressive blocking

  for (float blockRadius : blockRadii) {
    // Temporarily mark nodes near the blocked area as unwalkable
    std::vector<int> temporarilyBlockedNodes;

    for (size_t i = 0; i < nodes.size(); ++i) {
      float distance = Vector3Distance(nodes[i].position, blockedArea);
      if (distance <= blockRadius && nodes[i].walkable) {
        nodes[i].walkable = false;
        temporarilyBlockedNodes.push_back(i);
      }
    }

    // Find path with blocked area avoided
    std::vector<int> pathNodes = aStar(startNode, endNode);

    // Restore the temporarily blocked nodes
    for (int nodeIndex : temporarilyBlockedNodes) {
      if (nodeIndex >= 0 && nodeIndex < static_cast<int>(nodes.size())) {
        nodes[nodeIndex].walkable = true;
      }
    }

    // If we found a valid path with this block radius, use it
    if (!pathNodes.empty()) {
      std::vector<Vector3> path;
      for (int nodeIndex : pathNodes) {
        if (nodeIndex >= 0 && nodeIndex < static_cast<int>(nodes.size())) {
          path.push_back(nodes[nodeIndex].position);
        }
      }

      return path;
    }
  }

  // Strategy 3: Last resort - find any path that avoids the immediate blocked
  // area
  return findPath(start, end);
}

void NavMesh::rebuildConnections() {
  // Clear all existing connections
  for (auto& node : nodes) {
    node.connections.clear();
  }

  // Rebuild connections with proper line of sight checking
  connectNodes();
}

bool NavMesh::isNodeFullyAccessible(Vector3 nodePos, Vector3 entrancePos,
                                    Vector3 entranceSize) const {
  // Check if the node has enough clearance around it to be considered fully
  // accessible
  float clearanceRadius =
      nodeSpacing * 0.6f;  // Require 60% of node spacing as clearance

  // Sample points around the node to verify accessibility
  std::vector<Vector3> testPoints = {
      {nodePos.x - clearanceRadius, nodePos.y, nodePos.z},
      {nodePos.x + clearanceRadius, nodePos.y, nodePos.z},
      {nodePos.x, nodePos.y, nodePos.z - clearanceRadius},
      {nodePos.x, nodePos.y, nodePos.z + clearanceRadius},
      {nodePos.x - clearanceRadius * 0.7f, nodePos.y,
       nodePos.z - clearanceRadius * 0.7f},
      {nodePos.x + clearanceRadius * 0.7f, nodePos.y,
       nodePos.z - clearanceRadius * 0.7f},
      {nodePos.x - clearanceRadius * 0.7f, nodePos.y,
       nodePos.z + clearanceRadius * 0.7f},
      {nodePos.x + clearanceRadius * 0.7f, nodePos.y,
       nodePos.z + clearanceRadius * 0.7f}};

  // Calculate entrance boundaries
  float entranceHalfWidth = entranceSize.x / 2.0f;
  float leftBoundary = entrancePos.x - entranceHalfWidth;
  float rightBoundary = entrancePos.x + entranceHalfWidth;
  float frontBoundary = entrancePos.z + entranceSize.z / 2.0f;
  float backBoundary = entrancePos.z - entranceSize.z / 2.0f;

  // Check if all test points are within safe entrance bounds
  int validPoints = 0;
  for (const Vector3& testPoint : testPoints) {
    // Check if test point is within entrance corridor
    if (testPoint.x >= leftBoundary && testPoint.x <= rightBoundary &&
        testPoint.z >= backBoundary && testPoint.z <= frontBoundary + 2.0f) {
      validPoints++;
    }
  }

  // Node is fully accessible if at least 75% of test points are valid
  return validPoints >= static_cast<int>(testPoints.size() * 0.75f);
}

bool NavMesh::isNodeTooCloseToWall(Vector3 nodePos, Vector3 entrancePos,
                                   Vector3 entranceSize) const {
  // Check if node is too close to the front walls adjacent to the entrance
  float entranceHalfWidth = entranceSize.x / 2.0f;
  float wallThickness = 0.3f;                    // From Shop::buildWalls
  float minDistanceToWall = nodeSpacing * 0.5f;  // Minimum safe distance

  // Calculate wall boundaries
  float leftWallCenterX = entrancePos.x - entranceHalfWidth;
  float rightWallCenterX = entrancePos.x + entranceHalfWidth;
  float frontWallZ = entrancePos.z + entranceSize.z / 2.0f;

  // Check distance to left front wall edge
  if (nodePos.x <= leftWallCenterX + wallThickness / 2.0f &&
      nodePos.z <= frontWallZ + 1.0f) {
    float distToLeftWall =
        std::abs(nodePos.x - (leftWallCenterX + wallThickness / 2.0f));
    if (distToLeftWall < minDistanceToWall) {
      return true;
    }
  }

  // Check distance to right front wall edge
  if (nodePos.x >= rightWallCenterX - wallThickness / 2.0f &&
      nodePos.z <= frontWallZ + 1.0f) {
    float distToRightWall =
        std::abs(nodePos.x - (rightWallCenterX - wallThickness / 2.0f));
    if (distToRightWall < minDistanceToWall) {
      return true;
    }
  }

  return false;
}

void NavMesh::markNodesInArea(Vector3 center, Vector3 size, bool walkable,
                              float expansionFactor) {
  // Calculate expanded area with margin
  float margin = nodeSpacing * expansionFactor;
  Vector3 expandedSize = {size.x + margin * 2.0f, size.y,
                          size.z + margin * 2.0f};

  for (auto& node : nodes) {
    Vector3 diff = Vector3Subtract(node.position, center);

    // Check if node is within the expanded obstacle area (X and Z only)
    if (std::abs(diff.x) <= expandedSize.x / 2.0f &&
        std::abs(diff.z) <= expandedSize.z / 2.0f) {
      // Only affect ground-level nodes (nodes at NavMesh level)
      if (std::abs(node.position.y - (groundLevel + 0.5f)) < 0.2f) {
        node.walkable = walkable;
      }
    }
  }
}

void NavMesh::addShelfObstacle(Vector3 shelfPos, Vector3 shelfSize) {
  addObstacle(shelfPos, shelfSize, "shelf");
}

void NavMesh::addWallObstacle(Vector3 wallPos, Vector3 wallSize) {
  addObstacle(wallPos, wallSize, "wall");
}

void NavMesh::removeObstacle(Vector3 position, Vector3 size) {
  markNodesInArea(position, size, true, 0.5f);
  rebuildConnections();
}

bool NavMesh::isPositionBlocked(Vector3 position) const {
  int nearestNode = findNearestNode(position);
  if (nearestNode >= 0 && nearestNode < nodes.size()) {
    return !nodes[nearestNode].walkable;
  }
  return true;  // Assume blocked if no valid node found
}

std::vector<int> NavMesh::smoothPath(
    const std::vector<int>& originalPath) const {
  if (originalPath.size() <= 2) {
    return originalPath;
  }

  std::vector<int> smoothedPath;
  smoothedPath.push_back(originalPath[0]);  // Always include start

  int currentIndex = 0;

  while (currentIndex < static_cast<int>(originalPath.size()) - 1) {
    int furthestReachable = currentIndex + 1;

    // Find the furthest node we can reach directly from current position
    for (int i = currentIndex + 2; i < static_cast<int>(originalPath.size());
         ++i) {
      Vector3 currentPos = nodes[originalPath[currentIndex]].position;
      Vector3 targetPos = nodes[originalPath[i]].position;

      if (hasLineOfSight(currentPos, targetPos)) {
        furthestReachable = i;
      } else {
        break;  // Stop at first unreachable node
      }
    }

    // Add the furthest reachable node
    if (furthestReachable < static_cast<int>(originalPath.size()) - 1) {
      smoothedPath.push_back(originalPath[furthestReachable]);
    }

    currentIndex = furthestReachable;
  }

  // Always include end
  if (smoothedPath.back() != originalPath.back()) {
    smoothedPath.push_back(originalPath.back());
  }

  return smoothedPath;
}

// Professional object management system
void NavMesh::registerObject(GameObject* object) {
  if (!object || objectSet.find(object) != objectSet.end()) {
    return;  // Already registered or invalid object
  }

  registeredObjects.emplace_back(object);
  objectSet.insert(object);

  // Calculate blocking ONCE at registration time
  std::string objectType = object->getObstacleType();
  bool shouldBlock = shouldObjectBlockPath(object);

  // Update the registered object info
  auto& regObj = registeredObjects.back();
  regObj.lastPosition = object->getPosition();
  regObj.lastSize = object->getObstacleSize();
  regObj.wasBlocking = shouldBlock;

  // Apply blocking immediately for static objects
  if (shouldBlock) {
    markObjectNodes(object, false);  // Mark as unwalkable
  }
}

void NavMesh::unregisterObject(GameObject* object) {
  if (!object) return;

  // Remove from set first
  objectSet.erase(object);

  // Find and remove from vector
  auto it = std::find_if(
      registeredObjects.begin(), registeredObjects.end(),
      [object](const RegisteredObject& reg) { return reg.object == object; });

  if (it != registeredObjects.end()) {
    // If object was blocking, mark its nodes as walkable before removing
    if (it->wasBlocking) {
      markObjectNodes(object, true);
      // Only rebuild if we actually changed something
      rebuildConnections();
    }
    registeredObjects.erase(it);
  }
}

void NavMesh::updateAllRegisteredObjects() {
  // For performance: static objects are NEVER updated after initial setup
  // NPCs don't need pathfinding updates - they are too small to block
  // significantly

  // Only clean up invalid objects
  auto it = registeredObjects.begin();
  while (it != registeredObjects.end()) {
    try {
      // Just verify object is still valid, don't update positions
      it->object->getPosition();  // This will throw if invalid
      ++it;
    } catch (...) {
      // Object is invalid, remove it
      objectSet.erase(it->object);
      it = registeredObjects.erase(it);
    }
  }
}

void NavMesh::updateObjectBlocking(GameObject* object) {
  if (!object) return;

  // Find the registered object
  auto it = std::find_if(
      registeredObjects.begin(), registeredObjects.end(),
      [object](const RegisteredObject& reg) { return reg.object == object; });

  if (it == registeredObjects.end()) {
    return;  // Object not registered
  }

  Vector3 currentPos = object->getPosition();
  Vector3 currentSize = object->getObstacleSize();
  bool currentlyBlocking = shouldObjectBlockPath(object);

  // If blocking status changed, update the nodes
  if (it->wasBlocking != currentlyBlocking) {
    if (it->wasBlocking && !currentlyBlocking) {
      // Object no longer blocks, mark old position nodes as walkable
      markNodesInArea(it->lastPosition, it->lastSize, true);
    }

    if (!it->wasBlocking && currentlyBlocking) {
      // Object now blocks, mark nodes underneath as unwalkable
      markObjectNodes(object, false);
    }
  } else if (currentlyBlocking) {
    // Object still blocks but might have moved
    float positionDelta = Vector3Distance(currentPos, it->lastPosition);
    float sizeDelta = Vector3Distance(currentSize, it->lastSize);

    if (positionDelta > nodeSpacing * 0.1f || sizeDelta > nodeSpacing * 0.1f) {
      // Clear old position - make those nodes walkable again
      markNodesInArea(it->lastPosition, it->lastSize, true);
      // Mark new position nodes as unwalkable
      markObjectNodes(object, false);
    }
  } else if (!currentlyBlocking && it->wasBlocking) {
    // Object no longer blocks, clear any unwalkable nodes from old position
    markNodesInArea(it->lastPosition, it->lastSize, true);
  }

  // Update tracking information
  it->lastPosition = currentPos;
  it->lastSize = currentSize;
  it->wasBlocking = currentlyBlocking;
}

bool NavMesh::shouldObjectBlockPath(GameObject* object) const {
  if (!object) return false;

  std::string objectType = object->getObstacleType();

  // Floor objects should never block pathfinding - they are the walking surface
  if (objectType == "floor") {
    return false;
  }

  // NPC or other character bodies should not be considered obstacles for
  // pathfinding
  if (objectType == "npc" || objectType == "character" ||
      objectType == "player") {
    return false;
  }

  BoundingBox bbox = object->getBoundingBox();

  // If object is underground, it doesn't block
  if (bbox.max.y < groundLevel) {
    return false;
  }

  // Calculate the distance from ground to the bottom of the object
  float objectBottomY = bbox.min.y;
  float distanceFromGround = objectBottomY - groundLevel;

  // If the object starts below ground level but extends above it
  if (distanceFromGround < 0) {
    distanceFromGround = 0;
  }

  // Object blocks if the clearance under it is less than OR EQUAL to NPC height
  return distanceFromGround <= npcHeight;
}

float NavMesh::calculateBlockingHeight(GameObject* object) const {
  if (!object) return 0.0f;

  BoundingBox bbox = object->getBoundingBox();
  float objectBottomY = bbox.min.y;
  float distanceFromGround = objectBottomY - groundLevel;

  // Return the clearance height (how much space is available under the object)
  return std::max(0.0f, distanceFromGround);
}

void NavMesh::markObjectNodes(GameObject* object, bool walkable) {
  if (!object) return;

  Vector3 objPos = object->getPosition();
  Vector3 objSize = object->getObstacleSize();

  // Use a professional expansion factor based on object type
  float expansionFactor = 0.5f;
  std::string objectType = object->getObstacleType();

  if (objectType == "shelf") {
    expansionFactor = 0.6f;  // still bigger than default, but less aggressive
  } else if (objectType == "wall") {
    expansionFactor = 0.1f;  // minimal expansion for thin walls
  } else if (objectType == "character" || objectType == "npc") {
    expansionFactor = 0.2f;  // Minimal expansion for characters
  }

  markNodesInArea(objPos, objSize, walkable, expansionFactor);
}

void NavMesh::debugDrawRegisteredObjects() const {
  for (const auto& regObj : registeredObjects) {
    if (!regObj.object) continue;

    Vector3 pos = regObj.object->getPosition();
    Vector3 size = regObj.object->getObstacleSize();
    BoundingBox bbox = regObj.object->getBoundingBox();

    // Draw bounding box in different colors based on blocking status
    Color drawColor = regObj.wasBlocking ? RED : GREEN;
    drawColor.a = 100;  // Semi-transparent

    // Draw wireframe cube for the object
    DrawCubeWires(pos, size.x, size.y, size.z, drawColor);

    // Draw blocking height visualization
    if (regObj.wasBlocking) {
      float blockingHeight = calculateBlockingHeight(regObj.object);
      Vector3 groundPos = {pos.x, groundLevel, pos.z};
      Vector3 clearancePos = {pos.x, groundLevel + blockingHeight, pos.z};

      // Draw line from ground to clearance height
      DrawLine3D(groundPos, clearancePos, ORANGE);

      // Draw NPC height reference
      Vector3 npcHeightPos = {pos.x, groundLevel + npcHeight, pos.z};
      DrawLine3D({pos.x - 0.2f, groundLevel + npcHeight, pos.z},
                 {pos.x + 0.2f, groundLevel + npcHeight, pos.z}, BLUE);
    }
  }
}
