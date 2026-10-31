#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include "ai/NavMesh.h"
#include "common/TestAssert.h"

namespace {
struct SmokeOptions {
  int steps = 30000;
  int obstacleSets = 20;
  unsigned int seed = 1337;
};

void printUsage(const char* argv0) {
  std::cout << "Usage: " << argv0
            << " [--steps N] [--obstacle-sets N] [--seed N]\n";
}

bool parseArgs(int argc, char** argv, SmokeOptions& options) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--steps" && i + 1 < argc) {
      options.steps = std::max(100, std::atoi(argv[++i]));
    } else if (arg == "--obstacle-sets" && i + 1 < argc) {
      options.obstacleSets = std::max(1, std::atoi(argv[++i]));
    } else if (arg == "--seed" && i + 1 < argc) {
      options.seed = static_cast<unsigned int>(std::atoi(argv[++i]));
    } else if (arg == "--help" || arg == "-h") {
      printUsage(argv[0]);
      return false;
    } else {
      std::cerr << "Unknown argument: " << arg << "\n";
      printUsage(argv[0]);
      return false;
    }
  }
  return true;
}

Vector3 randomPoint(std::mt19937& rng, float min, float max) {
  std::uniform_real_distribution<float> dist(min, max);
  return {dist(rng), 0.5f, dist(rng)};
}

void applyObstacleSet(NavMesh& navMesh, std::mt19937& rng, int obstacleCount) {
  std::uniform_real_distribution<float> posDist(-20.0f, 20.0f);
  std::uniform_real_distribution<float> sizeDist(1.5f, 6.0f);
  std::bernoulli_distribution wallOrShelf(0.6);

  navMesh.generateNavMesh();

  for (int i = 0; i < obstacleCount; ++i) {
    Vector3 position = {posDist(rng), 0.5f, posDist(rng)};
    Vector3 size = {sizeDist(rng), 2.0f, sizeDist(rng)};
    navMesh.addObstacle(position, size, wallOrShelf(rng) ? "wall" : "shelf");
  }

  // Keep a guaranteed walkable corridor through the center.
  navMesh.ensureAreaWalkable({0.0f, 0.5f, 0.0f}, {6.0f, 1.0f, 40.0f});
  navMesh.rebuildConnections();
}

bool allWaypointsWalkable(const NavMesh& navMesh,
                          const std::vector<Vector3>& path) {
  return std::all_of(path.begin(), path.end(), [&](const Vector3& p) {
    return navMesh.isWalkable(p);
  });
}
}  // namespace

int main(int argc, char** argv) {
  SmokeOptions options;
  if (!parseArgs(argc, argv, options)) {
    return 1;
  }

  NavMesh navMesh({-25.0f, 0.0f, -25.0f}, {25.0f, 0.0f, 25.0f}, 0.8f, 0.0f);
  navMesh.generateNavMesh();

  std::mt19937 rng(options.seed);
  int successfulPaths = 0;
  int failedPaths = 0;
  int successfulAlternativePaths = 0;

  int stepsPerObstacleSet = std::max(1, options.steps / options.obstacleSets);
  for (int obstacleSet = 0; obstacleSet < options.obstacleSets; ++obstacleSet) {
    applyObstacleSet(navMesh, rng, 30);

    for (int i = 0; i < stepsPerObstacleSet; ++i) {
      Vector3 start = randomPoint(rng, -22.0f, 22.0f);
      Vector3 end = randomPoint(rng, -22.0f, 22.0f);

      std::vector<Vector3> path = navMesh.findPath(start, end);
      if (path.empty()) {
        failedPaths++;
      } else {
        successfulPaths++;
        TEST_ASSERT(allWaypointsWalkable(navMesh, path),
                    "Path contains non-walkable waypoints");
      }

      Vector3 blockedArea = randomPoint(rng, -12.0f, 12.0f);
      std::vector<Vector3> altPath =
          navMesh.findAlternativePath(start, end, blockedArea);
      if (!altPath.empty()) {
        successfulAlternativePaths++;
        TEST_ASSERT(allWaypointsWalkable(navMesh, altPath),
                    "Alternative path contains non-walkable waypoints");
      }
    }
  }

  TEST_ASSERT(successfulPaths > 0,
              "Smoke flow generated zero valid paths (navigation broken)");
  TEST_ASSERT(successfulAlternativePaths > 0,
              "Alternative path search never succeeded");

  const int totalAttempts = successfulPaths + failedPaths;
  TEST_ASSERT(totalAttempts > 0, "No path attempts executed");

  const float successRate =
      static_cast<float>(successfulPaths) / static_cast<float>(totalAttempts);
  TEST_ASSERT(successRate > 0.35f,
              "Path success rate too low under stress conditions");

  std::cout << "SmokeFlowTest passed"
            << " | attempts=" << totalAttempts
            << " success=" << successfulPaths << " failed=" << failedPaths
            << " alt_success=" << successfulAlternativePaths
            << " success_rate=" << successRate << std::endl;

  return 0;
}

