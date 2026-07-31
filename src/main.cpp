#include <cstdlib>
#include <exception>
#include <iostream>

#include "environment/environment.hpp"
#include "solver/solver.hpp"

int main() {
  try {
    vbm::ConfigParser &parser = vbm::ConfigParser::getInstance();
    if (!parser.parse("config/settings.config")) {
      std::cout << "########################### Parsing results: ####"
                   "########################## \n";
      std::cout << "Error parsing config file" << std::endl;
      return EXIT_FAILURE;
    }
    std::cout << "########################### Parsing results: ####"
                 "########################## \n";
    std::cout << "Config file parsed successfully \n" << std::endl;
    auto config = parser.getConfig();

    vbm::Environment env(config);

    vbm::Solver sol(env);
    bool allSolversSucceeded = true;
    if (config.vstar && !sol.vStarSearch()) {
      allSolversSucceeded = false;
    }
    if (config.astar && !sol.aStarSearch()) {
      allSolversSucceeded = false;
    }
    if (config.distanceFunction && !sol.computeDistanceFunction()) {
      allSolversSucceeded = false;
    }
    if (config.visibilityBasedSolver && !sol.visibilityBasedSolver()) {
      allSolversSucceeded = false;
    }
    return allSolversSucceeded ? EXIT_SUCCESS : EXIT_FAILURE;
  } catch (const std::exception &error) {
    std::cerr << "Fatal error: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
