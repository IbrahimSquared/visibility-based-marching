#include <iostream>

#include "environment/environment.h"
#include "solver/solver.h"

using namespace vbs;

int main() {

  ConfigParser parser;
  if (!parser.parse("config/settings.config")) {
    std::cout << "########################### Parsing results: ####"
                 "########################## \n";
    std::cout << "Error parsing config file" << std::endl;
    return 1;
  } else {
    std::cout << "########################### Parsing results: ####"
                 "########################## \n";
    std::cout << "Config file parsed successfully \n" << std::endl;
  }
  auto config = parser.getConfig();

  environment env = environment(config);

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
  return 1;
}
