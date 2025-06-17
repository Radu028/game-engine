#include "ai/NavMesh.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <queue>

#include "raymath.h"

NavMesh::NavMesh(Vector3 minBounds, Vector3 maxBounds, float spacing)
    : minBounds(minBounds), maxBounds(maxBounds), nodeSpacing(spacing) {}

void NavMesh::generateNavMesh() {
  nodes.clear();

  for (float x = minBounds.x; x <= maxBounds.x; x += nodeSpacing) {
    for (float z = minBounds.z; z <= maxBounds.z; z += nodeSpacing) {
      Vector3 nodePos = {x, 0.5f, z};
      nodes.emplace_back(nodePos);
    }
  }

  connectNodes();
}

void NavMesh::connectNodes() {
  for (size_t i = 0; i < nodes.size(); ++i) {
    for (size_t j = i + 1; j < nodes.size(); ++j) {
      // Only connect walkable nodes
      if (!nodes[i].walkable || !nodes[j].walkable) {
        continue;
      }

      float distance = Vector3Distance(nodes[i].position, nodes[j].position);

      // Increased connection distance for better connectivity
      float maxConnectionDistance = nodeSpacing * 2.5f;  // Was 2.0f, now 2.5f

      if (distance <= maxConnectionDistance &&
          hasLineOfSight(nodes[i].position, nodes[j].position)) {
        nodes[i].connections.push_back(j);
        nodes[j].connections.push_back(i);
      }
    }
  }
}

// Obstacle management system
void NavMesh::addObstacle(Vector3 position, Vector3 size,
                          const std::string& type) {
  float expansionFactor = 0.3f;
  if (type == "shelf") {
    expansionFactor = 0.4f;
  } else if (type == "wall") {
    expansionFactor = 0.2f;
  }

  markNodesInArea(position, size, false, expansionFactor, type == "shelf");

  rebuildConnections();
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

  for (size_t i = 0; i < nodes.size(); ++i) {
    if (!nodes[i].walkable) continue;

    float distance = Vector3Distance(position, nodes[i].position);
    if (distance < minDistance) {
      minDistance = distance;
      nearestNode = static_cast<int>(i);
    }
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
  for (auto& node : nodes) {
    node.gCost = INFINITY;
    node.hCost = 0.0f;
    node.fCost = INFINITY;
    node.parent = -1;
  }

  auto compare = [this](int a, int b) {
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

  while (!openSet.empty()) {
    int current = openSet.top();
    openSet.pop();
    inOpenSet[current] = false;

    if (current == endNode) {
      std::vector<int> path;
      int node = endNode;
      while (node != -1) {
        path.push_back(node);
        node = nodes[node].parent;
      }
      std::reverse(path.begin(), path.end());
      return path;
    }

    inClosedSet[current] = true;

    for (int neighbor : nodes[current].connections) {
      if (!nodes[neighbor].walkable || inClosedSet[neighbor]) {
        continue;
      }

      float tentativeGCost =
          nodes[current].gCost + calculateDistance(current, neighbor);

      if (tentativeGCost < nodes[neighbor].gCost) {
        nodes[neighbor].parent = current;
        nodes[neighbor].gCost = tentativeGCost;
        nodes[neighbor].hCost = calculateHeuristic(neighbor, endNode);
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

bool NavMesh::hasLineOfSight(Vector3 from, Vector3 to) const {
  Vector3 direction = Vector3Subtract(to, from);
  float distance = Vector3Length(direction);

  if (distance < 0.1f) return true;

  direction = Vector3Normalize(direction);

  // Reduced sample rate for better performance and less restrictive checking
  int samples = static_cast<int>(distance / 1.0f) +
                1;  // Was 0.5f, now 1.0f for fewer samples

  // Limit maximum number of samples to prevent performance issues
  if (samples > 10) {
    samples = 10;
  }

  for (int i = 1; i < samples; ++i) {
    float t = static_cast<float>(i) / static_cast<float>(samples);
    Vector3 samplePoint =
        Vector3Add(from, Vector3Scale(direction, distance * t));

    // Direct node check instead of recursive isWalkable call
    int nearestNode = -1;
    float minDistance = INFINITY;

    for (size_t j = 0; j < nodes.size(); ++j) {
      float nodeDistance = Vector3Distance(samplePoint, nodes[j].position);
      if (nodeDistance < minDistance) {
        minDistance = nodeDistance;
        nearestNode = static_cast<int>(j);
      }
    }

    // More lenient distance threshold and walkability check
    if (nearestNode == -1 || minDistance > nodeSpacing * 2.0f ||
        !nodes[nearestNode].walkable) {
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

  // Try multiple alternative pathfinding strategies with different block radii
  std::vector<float> blockRadii = {
      2.0f, 1.5f, 1.0f, 0.5f};  // Try progressively smaller block areas

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

  return {};  // No alternative path found
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
                              float expansionFactor, bool projectToGround) {
  // Calculate expanded area with margin
  float margin = nodeSpacing * expansionFactor;
  Vector3 expandedSize = {size.x + margin * 2.0f, size.y,
                          size.z + margin * 2.0f};

  int nodesMarked = 0;

  for (auto& node : nodes) {
    Vector3 diff = Vector3Subtract(node.position, center);

    // Check if node is within the expanded obstacle area (X and Z only)
    // For floating obstacles, we project them down to ground level
    if (std::abs(diff.x) <= expandedSize.x / 2.0f &&
        std::abs(diff.z) <= expandedSize.z / 2.0f) {
      bool shouldMark = false;

      if (projectToGround) {
        if (node.position.y <= 0.6f) {
          shouldMark = true;
        }
      } else {
        if (std::abs(diff.y) <= expandedSize.y / 2.0f + 1.0f) {
          shouldMark = true;
        }
      }

      if (shouldMark) {
        // Special handling for shelves near boundaries - use asymmetric
        // expansion
        if (!walkable && projectToGround) {
          // Check if shelf is near shop back boundary (z < -11)
          if (center.z < -11.0f) {
            Vector3 asymmetricDiff = Vector3Subtract(node.position, center);

            // Reduce blocking area towards the back wall (negative Z direction)
            float backMargin =
                nodeSpacing * 0.3f;  // Reduced margin towards back wall
            float frontMargin =
                nodeSpacing * 1.0f;  // Increased margin towards accessible area

            bool inBackArea = (asymmetricDiff.z < 0 &&
                               std::abs(asymmetricDiff.z) <= backMargin);
            bool inFrontArea = (asymmetricDiff.z >= 0 &&
                                std::abs(asymmetricDiff.z) <= frontMargin);
            bool inSideArea =
                (std::abs(asymmetricDiff.x) <= nodeSpacing * expansionFactor);

            if ((inBackArea || inFrontArea) && inSideArea) {
              node.walkable = walkable;
              nodesMarked++;
            }
          } else {
            // For non-back-wall shelves, use standard marking
            node.walkable = walkable;
            nodesMarked++;
          }
        } else {
          // For non-shelf obstacles or walkable marking, use standard behavior
          node.walkable = walkable;
          nodesMarked++;
        }
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
  markNodesInArea(position, size, true, 0.5f, false);
  rebuildConnections();
}

bool NavMesh::isPositionBlocked(Vector3 position) const {
  int nearestNode = findNearestNode(position);
  if (nearestNode >= 0 && nearestNode < nodes.size()) {
    return !nodes[nearestNode].walkable;
  }
  return true;  // Assume blocked if no valid node found
}
