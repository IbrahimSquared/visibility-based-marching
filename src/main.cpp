#include <iostream>

#include "environment/environment.hpp"
#include "solver/solver.hpp"

int main() {
  vbs::ConfigParser &parser = vbs::ConfigParser::getInstance();
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

  vbs::Environment env = vbs::Environment(config);

  vbs::Solver sol = vbs::Solver(env);
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
