#include <iostream>
#include <vector>

#include "ai/NavMesh.h"
#include "common/TestAssert.h"

int main() {
  const Vector3 minBounds{-5.0f, 0.0f, -5.0f};
  const Vector3 maxBounds{5.0f, 0.0f, 5.0f};
  NavMesh navMesh(minBounds, maxBounds, 1.0f, 0.0f);
  navMesh.generateNavMesh();

  const Vector3 start{-4.0f, 0.5f, 0.0f};
  const Vector3 end{4.0f, 0.5f, 0.0f};

  std::vector<Vector3> baselinePath = navMesh.findPath(start, end);
  TEST_ASSERT(!baselinePath.empty(), "Baseline path should exist");

  // Block almost the entire field to force no-path.
  navMesh.addObstacle({0.0f, 0.5f, 0.0f}, {100.0f, 1.0f, 100.0f}, "wall");
  navMesh.rebuildConnections();
  std::vector<Vector3> blockedPath = navMesh.findPath(start, end);
  TEST_ASSERT(blockedPath.empty(), "Path should be blocked after full obstacle");

  // Re-open a corridor through the blocked area.
  navMesh.ensureAreaWalkable({0.0f, 0.5f, 0.0f}, {10.0f, 1.0f, 1.0f});
  navMesh.rebuildConnections();
  std::vector<Vector3> restoredPath = navMesh.findPath(start, end);
  TEST_ASSERT(!restoredPath.empty(),
              "Path should be restored after corridor reopening");

  // Adding floor as obstacle should be ignored by navmesh.
  navMesh.addObstacle({0.0f, 0.5f, 0.0f}, {8.0f, 1.0f, 8.0f}, "floor");
  navMesh.rebuildConnections();
  std::vector<Vector3> afterFloorPath = navMesh.findPath(start, end);
  TEST_ASSERT(!afterFloorPath.empty(),
              "Floor obstacle marker should not block navigation");

  // Ensure diagonal paths cannot cut through blocked corners.
  NavMesh cornerMesh({-1.0f, 0.0f, -1.0f}, {0.0f, 0.0f, 0.0f}, 1.0f, 0.0f);
  cornerMesh.generateNavMesh();
  cornerMesh.addObstacle({0.0f, 0.5f, -1.0f}, {0.2f, 2.0f, 0.2f}, "wall");
  cornerMesh.addObstacle({-1.0f, 0.5f, 0.0f}, {0.2f, 2.0f, 0.2f}, "wall");
  cornerMesh.rebuildConnections();

  std::vector<Vector3> cornerCutPath =
      cornerMesh.findPath({-1.0f, 0.5f, -1.0f}, {0.0f, 0.5f, 0.0f});
  TEST_ASSERT(cornerCutPath.empty(),
              "Path should not cut diagonally through blocked corners");

  std::cout << "NavMeshTests passed" << std::endl;
  return 0;
}
