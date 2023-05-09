#include "environment/environment.h"
#include "solver/solver.h"

#include <iostream>

using namespace vbs;

int main() {
  // Parse settings
  ConfigParser parser;
  parser.parse("config/settings.config");
  auto config = parser.getConfig();

  // Initialize environment
  environment env = environment(config);

  // Initialize solver & solve
  solver sol = solver(env);
  if (config.vstar) {
    sol.vStarSearch();
  }
  if (config.astar) {
    sol.aStarSearch();
  }
  if (config.distanceFunction) {
    sol.computeDistanceFunction();
  }
  if (config.visibilityBasedSolver) {
    sol.visibilityBasedSolver();
  }
}