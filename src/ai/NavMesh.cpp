#include "ai/NavMesh.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <queue>

#include "objects/GameObject.h"
#include "raymath.h"
#include "settings/Physics.h"

NavMesh::NavMesh(Vector3 minBounds, Vector3 maxBounds, float spacing,
                 float groundY)
    : nodeSpacing(spacing),
      minBounds(minBounds),
      maxBounds(maxBounds),
      groundLevel(groundY),
      npcHeight(GameSettings::Character::HEIGHT),
      gridWidth(0),
      gridHeight(0) {}

void NavMesh::setNodeWalkable(int nodeIndex, bool walkable) {
  if (nodeIndex < 0 || nodeIndex >= static_cast<int>(nodes.size())) return;
  if (nodes[nodeIndex].walkable == walkable) return;

  nodes[nodeIndex].walkable = walkable;

  if (!walkable) {
    if (blockedNodeLookup.insert(nodeIndex).second) {
      blockedNodeCache.push_back(nodeIndex);
    }
  } else {
    if (blockedNodeLookup.erase(nodeIndex) > 0) {
      auto it = std::find(blockedNodeCache.begin(), blockedNodeCache.end(),
                          nodeIndex);
      if (it != blockedNodeCache.end()) {
        blockedNodeCache.erase(it);
      }
    }
  }
}

void NavMesh::generateNavMesh() {
  nodes.clear();
  blockedNodeLookup.clear();
  blockedNodeCache.clear();

  float widthSpan = maxBounds.x - minBounds.x;
  float heightSpan = maxBounds.z - minBounds.z;

  gridWidth =
      std::max(1, static_cast<int>(std::floor(widthSpan / nodeSpacing)) + 1);
  gridHeight =
      std::max(1, static_cast<int>(std::floor(heightSpan / nodeSpacing)) + 1);

  nodes.reserve(static_cast<size_t>(gridWidth) *
                static_cast<size_t>(gridHeight));

  for (int z = 0; z < gridHeight; ++z) {
    float zPos = minBounds.z + static_cast<float>(z) * nodeSpacing;
    for (int x = 0; x < gridWidth; ++x) {
      float xPos = minBounds.x + static_cast<float>(x) * nodeSpacing;
      Vector3 nodePos = {xPos, groundLevel + 0.5f, zPos};
      nodes.emplace_back(nodePos);
    }
  }

  connectNodes();
}

void NavMesh::connectNodes() {
  for (auto& node : nodes) {
    node.connections.clear();
  }

  if (gridWidth <= 0 || gridHeight <= 0) return;

  for (size_t i = 0; i < nodes.size(); ++i) {
    if (!nodes[i].walkable) continue;

    int gridZ = static_cast<int>(i / gridWidth);
    int gridX = static_cast<int>(i % gridWidth);

    for (int dz = -1; dz <= 1; ++dz) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dz == 0) continue;

        int neighborIndex = gridCoordsToIndex(gridX + dx, gridZ + dz);
        if (neighborIndex == -1) continue;
        if (!nodes[neighborIndex].walkable) continue;

        // Prevent corner-cutting: diagonal traversal is valid only if both
        // orthogonal neighbors are also walkable.
        if (dx != 0 && dz != 0) {
          int sideNeighborA = gridCoordsToIndex(gridX + dx, gridZ);
          int sideNeighborB = gridCoordsToIndex(gridX, gridZ + dz);
          if (sideNeighborA == -1 || sideNeighborB == -1) continue;
          if (!nodes[sideNeighborA].walkable || !nodes[sideNeighborB].walkable)
            continue;
        }

        nodes[i].connections.push_back(neighborIndex);
      }
    }
  }
}

int NavMesh::gridCoordsToIndex(int x, int z) const {
  if (x < 0 || x >= gridWidth || z < 0 || z >= gridHeight) return -1;

  return z * gridWidth + x;
}

int NavMesh::positionToIndex(Vector3 position) const {
  if (gridWidth <= 0 || gridHeight <= 0 || nodeSpacing <= 0.0f) return -1;

  float localX = (position.x - minBounds.x) / nodeSpacing;
  float localZ = (position.z - minBounds.z) / nodeSpacing;

  int x = static_cast<int>(std::round(localX));
  int z = static_cast<int>(std::round(localZ));

  return gridCoordsToIndex(x, z);
}

void NavMesh::addObstacle(Vector3 position, Vector3 size,
                          const std::string& type) {
  // Minimal expansion factors for precise obstacle marking
  float expansionFactor = 0.1f;  // Very small default expansion

  if (type == "shelf") {
    // Shelves need a small buffer for NPC to stand next to them
    expansionFactor = 0.2f;
  } else if (type == "wall") {
    // Walls: no expansion at all - only exact bounds
    expansionFactor = 0.0f;
  } else if (type == "floor") {
    // Floors should not block anything
    return;
  }

  markNodesInArea(position, size, false, expansionFactor);

  // Don't rebuild connections here - do it once at the end in
  // initializeNavMesh()
}

std::vector<Vector3> NavMesh::findPath(Vector3 start, Vector3 end) {
  auto findBestNodeForPoint = [&](Vector3 point) -> int {
    int nearestNode = -1;
    float nearestDistance = INFINITY;
    int nearestVisibleNode = -1;
    float nearestVisibleDistance = INFINITY;
    const float maxVisibleSearchDistance = nodeSpacing * 6.0f;

    for (size_t i = 0; i < nodes.size(); ++i) {
      if (!nodes[i].walkable) continue;

      float distance = Vector3Distance(point, nodes[i].position);
      if (distance < nearestDistance) {
        nearestDistance = distance;
        nearestNode = static_cast<int>(i);
      }

      if (distance > maxVisibleSearchDistance ||
          distance >= nearestVisibleDistance) {
        continue;
      }

      if (!hasLineOfSight(point, nodes[i].position)) {
        continue;
      }

      if (distance < nearestVisibleDistance) {
        nearestVisibleDistance = distance;
        nearestVisibleNode = static_cast<int>(i);
      }
    }

    if (nearestVisibleNode != -1) return nearestVisibleNode;
    return nearestNode;
  };

  int startNode = findBestNodeForPoint(start);
  int endNode = findBestNodeForPoint(end);

  if (startNode == -1 || endNode == -1) {
    return {};
  }

  if (!nodes[startNode].walkable || !nodes[endNode].walkable) {
    return {};
  }

  if (startNode == endNode) {
    return {nodes[static_cast<size_t>(startNode)].position};
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
  int nodeIndex = positionToIndex(position);
  if (nodeIndex == -1) return false;

  float distance = Vector3Distance(position, nodes[nodeIndex].position);
  if (distance > nodeSpacing * 1.5f) return false;

  return nodes[nodeIndex].walkable;
}

std::vector<int> NavMesh::aStar(int startNode, int endNode) {
  if (nodes.empty()) {
    return {};
  }

  // Reset search state
  for (auto& node : nodes) {
    node.gCost = INFINITY;
    node.hCost = 0.0f;
    node.fCost = INFINITY;
    node.parent = -1;
  }

  auto compare = [this](int lhs, int rhs) {
    return nodes[lhs].fCost > nodes[rhs].fCost;
  };

  std::priority_queue<int, std::vector<int>, decltype(compare)> openSet(
      compare);
  std::vector<bool> closedSet(nodes.size(), false);

  nodes[startNode].gCost = 0.0f;
  nodes[startNode].hCost = calculateHeuristic(startNode, endNode);
  nodes[startNode].fCost = nodes[startNode].gCost + nodes[startNode].hCost;

  openSet.push(startNode);

  auto reconstructPath = [&](int node) {
    std::vector<int> path;
    while (node != -1) {
      path.push_back(node);
      node = nodes[node].parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
  };

  while (!openSet.empty()) {
    int current = openSet.top();
    openSet.pop();

    if (closedSet[current]) {
      continue;
    }

    closedSet[current] = true;

    if (current == endNode) {
      return reconstructPath(endNode);
    }

    for (int neighbor : nodes[current].connections) {
      if (!nodes[neighbor].walkable || closedSet[neighbor]) {
        continue;
      }

      float stepCost = calculateDistance(current, neighbor);
      float movementPenalty =
          calculateMovementPenalty(current, neighbor, endNode);
      float tentativeG = nodes[current].gCost + stepCost + movementPenalty;

      if (tentativeG >= nodes[neighbor].gCost) {
        continue;
      }

      nodes[neighbor].parent = current;
      nodes[neighbor].gCost = tentativeG;
      nodes[neighbor].hCost = calculateImprovedHeuristic(neighbor, endNode);
      nodes[neighbor].fCost = nodes[neighbor].gCost + nodes[neighbor].hCost;

      openSet.push(neighbor);
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
  float opennessPenalty = 0.0f;
  int connectionCount = static_cast<int>(nodes[nodeA].connections.size());
  if (connectionCount < 4) {
    opennessPenalty = (4 - connectionCount) * nodeSpacing * 0.05f;
  }

  float straightnessPenalty = 0.0f;
  if (nodes[nodeA].parent != -1 && euclideanDistance > 0.0f) {
    Vector3 prevDirection =
        Vector3Subtract(posA, nodes[nodes[nodeA].parent].position);
    if (Vector3Length(prevDirection) > 0.1f) {
      prevDirection = Vector3Normalize(prevDirection);
      Vector3 direction = Vector3Normalize(diff);
      float dot = Vector3DotProduct(direction, prevDirection);
      straightnessPenalty = (1.0f - dot) * 0.2f;
    }
  }

  return euclideanDistance + opennessPenalty + straightnessPenalty;
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

  direction = Vector3Normalize(direction);

  // Sample every half-node for precise collision detection
  float stepSize = nodeSpacing * 0.5f;
  int samples = std::max(2, static_cast<int>(distance / stepSize) + 1);

  for (int i = 1; i < samples; ++i) {
    float t = static_cast<float>(i) / static_cast<float>(samples);
    Vector3 samplePoint =
        Vector3Add(from, Vector3Scale(direction, distance * t));

    int sampleIndex = positionToIndex(samplePoint);
    if (sampleIndex == -1 || !nodes[sampleIndex].walkable) {
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
        setNodeWalkable(static_cast<int>(i), false);
        temporarilyBlockedNodes.push_back(static_cast<int>(i));
      }
    }

    // Find path with blocked area avoided
    std::vector<int> pathNodes = aStar(startNode, endNode);

    // Restore the temporarily blocked nodes
    for (int nodeIndex : temporarilyBlockedNodes) {
      setNodeWalkable(nodeIndex, true);
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

void NavMesh::markNodesInArea(Vector3 center, Vector3 size, bool walkable,
                              float expansionFactor) {
  // Keep a safety buffer around obstacles so generated paths match the actual
  // physical footprint of the NPC.
  const float npcClearance = std::max(0.1f, GameSettings::Character::RADIUS * 0.75f);
  float margin = nodeSpacing * expansionFactor + npcClearance;

  float effectiveMarginX = margin;
  float effectiveMarginZ = margin;

  // Thin obstacles still need clearance, but reduce it slightly to avoid
  // over-blocking narrow corridors.
  if (size.x < 0.5f) effectiveMarginX = margin * 0.75f;
  if (size.z < 0.5f) effectiveMarginZ = margin * 0.75f;

  float halfSizeX = size.x / 2.0f + effectiveMarginX;
  float halfSizeZ = size.z / 2.0f + effectiveMarginZ;

  for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
    const auto& node = nodes[static_cast<size_t>(i)];
    Vector3 diff = Vector3Subtract(node.position, center);

    // Check if node is within the expanded obstacle area (X and Z only).
    if (std::abs(diff.x) <= halfSizeX && std::abs(diff.z) <= halfSizeZ) {
      // Only affect ground-level nodes (nodes at NavMesh level)
      if (std::abs(node.position.y - (groundLevel + 0.5f)) < 0.5f) {
        setNodeWalkable(i, walkable);
      }
    }
  }
}

void NavMesh::removeObstacle(Vector3 position, Vector3 size) {
  markNodesInArea(position, size, true, 0.5f);
  rebuildConnections();
}

void NavMesh::ensureAreaWalkable(Vector3 center, Vector3 size) {
  // Force mark an area as walkable (useful for entrances/corridors)
  // This overrides any obstacle marking in this area
  float halfX = size.x / 2.0f;
  float halfZ = size.z / 2.0f;

  for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
    const auto& node = nodes[static_cast<size_t>(i)];
    Vector3 diff = Vector3Subtract(node.position, center);

    // Check if node is within the area (X and Z only)
    // Use <= to include boundary nodes
    if (std::abs(diff.x) <= halfX && std::abs(diff.z) <= halfZ) {
      // Only affect ground-level nodes
      if (std::abs(node.position.y - (groundLevel + 0.5f)) < 1.0f) {
        if (!nodes[i].walkable) {
          setNodeWalkable(i, true);
        }
      }
    }
  }
}

bool NavMesh::isPositionBlocked(Vector3 position) const {
  int nearestNode = findNearestNode(position);
  if (nearestNode >= 0 && nearestNode < static_cast<int>(nodes.size())) {
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

  // Pickups/items should not be treated as obstacles.
  if (objectType == "pickup" || objectType == "item") {
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
  std::string objectType = object->getObstacleType();

  // Skip floor objects entirely
  if (objectType == "floor") {
    return;
  }

  // Skip characters/NPCs - they handle their own collision
  if (objectType == "character" || objectType == "npc") {
    return;
  }

  // Minimal expansion for precise obstacle bounds
  float expansionFactor = 0.1f;

  if (objectType == "shelf") {
    expansionFactor = 0.15f;  // Small buffer for standing next to shelves
  } else if (objectType == "wall") {
    expansionFactor = 0.0f;  // Exact wall bounds only
  }

  markNodesInArea(objPos, objSize, walkable, expansionFactor);
}

void NavMesh::debugDrawRegisteredObjects() const {
  for (const auto& regObj : registeredObjects) {
    if (!regObj.object) continue;

    Vector3 pos = regObj.object->getPosition();
    Vector3 size = regObj.object->getObstacleSize();

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
      DrawLine3D({pos.x - 0.2f, groundLevel + npcHeight, pos.z},
                 {pos.x + 0.2f, groundLevel + npcHeight, pos.z}, BLUE);
    }
  }
}
