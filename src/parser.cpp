#include "parser/parser.hpp"

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace vbm {

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
bool ConfigParser::parse(const std::string &filename) {
  std::ifstream file(filename);
  if (!file) {
    std::cerr << "Failed to open " << filename << '\n';
    return false;
  }
  config_ = Config{};
  bool initialFrontlineProvided = false;
  bool targetXProvided = false;
  bool targetYProvided = false;

  std::cout << "########################### Parsing config file  "
               "########################## \n";
  std::cout << "Attempting to parse " << filename << '\n';
  std::string line;
  while (std::getline(file, line)) {
    // Ignore comments and blank lines
    if (line.empty() || line[0] == '#') {
      continue;
    }

    // Split the line into key and value
    std::istringstream iss(line);
    std::string key;
    if (!std::getline(iss, key, '=')) {
      continue;
    }
    std::string value;
    if (!std::getline(iss, value)) {
      continue;
    }

    // Trim leading and trailing whitespace from key and value
    key.erase(0, key.find_first_not_of(" \t"));
    key.erase(key.find_last_not_of(" \t") + 1);
    value.erase(0, value.find_first_not_of(" \t"));
    value.erase(value.find_last_not_of(" \t") + 1);

    // Parse the value based on the key's data type
    if (key == "mode") {
      try {
        config_.mode = std::stoi(value);
        if (config_.mode != 1 && config_.mode != 2) {
          std::cerr << "Invalid value for " << key << ": " << value
                    << ", using default value 1\n";
          config_.mode = 1;
        }
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be an integer\n";
        return false;
      }
    } else if (key == "ncols") {
      try {
        const int parsedValue = std::stoi(value);
        if (parsedValue <= 0) {
          std::cerr << "Invalid value for " << key << ": " << value << '\n';
          std::cerr << "It must be a positive integer\n";
          return false;
        }
        config_.ncols = parsedValue;
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a positive integer\n";
        return false;
      }
    } else if (key == "nrows") {
      try {
        const int parsedValue = std::stoi(value);
        if (parsedValue <= 0) {
          std::cerr << "Invalid value for " << key << ": " << value << '\n';
          std::cerr << "It must be a positive integer\n";
          return false;
        }
        config_.nrows = parsedValue;
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a positive integer\n";
        return false;
      }
    } else if (key == "nb_of_obstacles") {
      try {
        const int parsedValue = std::stoi(value);
        if (parsedValue < 0) {
          std::cerr << "Invalid value for " << key << ": " << value << '\n';
          std::cerr << "It must be a non-negative integer\n";
          return false;
        }
        config_.nb_of_obstacles = parsedValue;
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a non-negative integer\n";
        return false;
      }
    } else if (key == "minWidth") {
      try {
        const int parsedValue = std::stoi(value);
        if (parsedValue <= 0) {
          std::cerr << "Invalid value for " << key << ": " << value << '\n';
          std::cerr << "It must be a positive integer\n";
          return false;
        }
        config_.minWidth = parsedValue;
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a positive integer\n";
        return false;
      }
    } else if (key == "maxWidth") {
      try {
        const int parsedValue = std::stoi(value);
        if (parsedValue <= 0) {
          std::cerr << "Invalid value for " << key << ": " << value << '\n';
          std::cerr << "It must be a positive integer\n";
          return false;
        }
        config_.maxWidth = parsedValue;
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a positive integer\n";
        return false;
      }
    } else if (key == "minHeight") {
      try {
        const int parsedValue = std::stoi(value);
        if (parsedValue <= 0) {
          std::cerr << "Invalid value for " << key << ": " << value << '\n';
          std::cerr << "It must be a positive integer\n";
          return false;
        }
        config_.minHeight = parsedValue;
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a positive integer\n";
        return false;
      }
    } else if (key == "maxHeight") {
      try {
        const int parsedValue = std::stoi(value);
        if (parsedValue <= 0) {
          std::cerr << "Invalid value for " << key << ": " << value << '\n';
          std::cerr << "It must be a positive integer\n";
          return false;
        }
        config_.maxHeight = parsedValue;
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a positive integer\n";
        return false;
      }
    } else if (key == "randomSeed") {
      if (value == "0" || value == "false") {
        config_.randomSeed = false;
      } else if (value == "1" || value == "true") {
        config_.randomSeed = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "seedValue") {
      try {
        config_.seedValue = std::stoi(value);
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be an integer\n";
        return false;
      }
    } else if (key == "imagePath") {
      config_.imagePath = value;
    } else if (key == "greedy") {
      if (value == "0" || value == "false") {
        config_.greedy = false;
      } else if (value == "1" || value == "true") {
        config_.greedy = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "visibilityBasedSolver") {
      if (value == "0" || value == "false") {
        config_.visibilityBasedSolver = false;
      } else if (value == "1" || value == "true") {
        config_.visibilityBasedSolver = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "msfm") {
      if (value == "0" || value == "false") {
        config_.msfm = false;
      } else if (value == "1" || value == "true") {
        config_.msfm = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "expandInObstacles") {
      if (value == "0" || value == "false") {
        config_.expandInObstacles = false;
      } else if (value == "1" || value == "true") {
        config_.expandInObstacles = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "vstar") {
      if (value == "0" || value == "false") {
        config_.vstar = false;
      } else if (value == "1" || value == "true") {
        config_.vstar = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "astar") {
      if (value == "0" || value == "false") {
        config_.astar = false;
      } else if (value == "1" || value == "true") {
        config_.astar = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "distanceFunction") {
      if (value == "0" || value == "false") {
        config_.distanceFunction = false;
      } else if (value == "1" || value == "true") {
        config_.distanceFunction = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "initialFrontline") {
      try {
        config_.initialFrontline = parseVectorString(value);
        initialFrontlineProvided = true;
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a brace-enclosed list of integers\n";
        return false;
      }
    } else if (key == "target_x") {
      try {
        config_.target_x = std::stoi(value);
        targetXProvided = true;
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a positive integer\n";
        return false;
      }
    } else if (key == "target_y") {
      try {
        config_.target_y = std::stoi(value);
        targetYProvided = true;
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a positive integer\n";
        return false;
      }
    } else if (key == "visibilityThreshold") {
      try {
        const double parsedValue = std::stod(value);
        if (!std::isfinite(parsedValue) || parsedValue > 1.0 ||
            parsedValue < 0) {
          std::cerr << "Invalid value for " << key << ": " << value << '\n';
          std::cerr << "It must be a finite double between 0 and 1\n";
          return false;
        }
        config_.visibilityThreshold = parsedValue;
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a finite double between 0 and 1\n";
        return false;
      }
    } else if (key == "speedValue") {
      try {
        const float parsedValue = std::stof(value);
        if (!std::isfinite(parsedValue) || parsedValue <= 0) {
          std::cerr << "Invalid value for " << key << ": " << value << '\n';
          std::cerr << "It must be a finite positive number\n";
          return false;
        }
        config_.speedValue = parsedValue;
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a finite positive number\n";
        return false;
      }
    } else if (key == "timer") {
      if (value == "0" || value == "false") {
        config_.timer = false;
      } else if (value == "1" || value == "true") {
        config_.timer = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "saveResults") {
      if (value == "0" || value == "false") {
        config_.saveResults = false;
      } else if (value == "1" || value == "true") {
        config_.saveResults = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "saveCameFrom") {
      if (value == "0" || value == "false") {
        config_.saveCameFrom = false;
      } else if (value == "1" || value == "true") {
        config_.saveCameFrom = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "saveGScore") {
      if (value == "0" || value == "false") {
        config_.saveGScore = false;
      } else if (value == "1" || value == "true") {
        config_.saveGScore = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "saveLightSources") {
      if (value == "0" || value == "false") {
        config_.saveLightSources = false;
      } else if (value == "1" || value == "true") {
        config_.saveLightSources = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "saveVisibilityField") {
      if (value == "0" || value == "false") {
        config_.saveVisibilityField = false;
      } else if (value == "1" || value == "true") {
        config_.saveVisibilityField = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "saveSpeedField") {
      if (value == "0" || value == "false") {
        config_.saveSpeedField = false;
      } else if (value == "1" || value == "true") {
        config_.saveSpeedField = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "savePath") {
      if (value == "0" || value == "false") {
        config_.savePath = false;
      } else if (value == "1" || value == "true") {
        config_.savePath = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "saveFScore") {
      if (value == "0" || value == "false") {
        config_.saveFScore = false;
      } else if (value == "1" || value == "true") {
        config_.saveFScore = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "saveDistanceFunction") {
      if (value == "0" || value == "false") {
        config_.saveDistanceFunction = false;
      } else if (value == "1" || value == "true") {
        config_.saveDistanceFunction = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "silent") {
      if (value == "0" || value == "false") {
        config_.silent = false;
      } else if (value == "1" || value == "true") {
        config_.silent = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "saveVisibilityBasedSolverImage") {
      if (value == "0" || value == "false") {
        config_.saveVisibilityBasedSolverImage = false;
      } else if (value == "1" || value == "true") {
        config_.saveVisibilityBasedSolverImage = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "saveDistanceFunctionImage") {
      if (value == "0" || value == "false") {
        config_.saveDistanceFunctionImage = false;
      } else if (value == "1" || value == "true") {
        config_.saveDistanceFunctionImage = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "ballRadius") {
      try {
        config_.ballRadius = std::stoi(value);
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be an integer\n";
        return false;
      }
    } else if (key == "number_of_contour_lines") {
      try {
        config_.number_of_contour_lines = std::stoi(value);
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be an integer\n";
        return false;
      }
    } else if (key == "contourLines") {
      if (value == "0" || value == "false") {
        config_.contourLines = false;
      } else if (value == "1" || value == "true") {
        config_.contourLines = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else {
      std::cerr << "Invalid/Irrelevant key: " << key << '\n';
      std::cerr << "Ignoring...\n";
    }
  }
  if (config_.mode == 1 && config_.minWidth > config_.maxWidth) {
    std::cerr << "minWidth must be less than or equal to maxWidth in random "
                 "environment mode\n";
    return false;
  }
  if (config_.mode == 1 && config_.minHeight > config_.maxHeight) {
    std::cerr << "minHeight must be less than or equal to maxHeight in random "
                 "environment mode\n";
    return false;
  }

  const bool pointToPointSolverEnabled = config_.vstar || config_.astar;
  if (pointToPointSolverEnabled &&
      (!targetXProvided || !targetYProvided)) {
    std::cerr << "target_x and target_y are required when VStar or AStar is "
                 "enabled\n";
    return false;
  }
  if (pointToPointSolverEnabled &&
      (!initialFrontlineProvided || config_.initialFrontline.size() != 2)) {
    std::cerr << "initialFrontline must contain exactly one coordinate pair "
                 "when VStar or AStar is enabled\n";
    return false;
  }
  if (config_.visibilityBasedSolver &&
      (!initialFrontlineProvided || config_.initialFrontline.empty() ||
       config_.initialFrontline.size() % 2 != 0)) {
    std::cerr << "initialFrontline must contain one or more coordinate pairs "
                 "when the visibility-based solver is enabled\n";
    return false;
  }

  if (!config_.silent) {
    if (config_.mode == 1) {
      std::cout << "Random environment mode" << std::endl;
      std::cout << "########################### Environment settings "
                   "########################## \n"
                << "ncols: " << config_.ncols << "\n"
                << "nrows: " << config_.nrows << "\n"
                << "Nb of obstacles: " << config_.nb_of_obstacles << "\n"
                << "Min width: " << config_.minWidth << "\n"
                << "Max width: " << config_.maxWidth << "\n"
                << "Min height: " << config_.minHeight << "\n"
                << "Max height: " << config_.maxHeight << "\n"
                << "Speed value: " << config_.speedValue << std::endl;
      if (config_.randomSeed) {
        std::cout << "Random seed: " << config_.randomSeed << std::endl;
      } else {
        std::cout << "Fixed seed value: " << config_.seedValue << std::endl;
      }
    } else if (config_.mode == 2) {
      std::cout << "Import image mode"
                << "\n"
                << "Image path: " << config_.imagePath << std::endl;
    }
    std::cout
        << "############################ Solver settings "
           "############################## \n"
        << "Solver visibility threshold: " << config_.visibilityThreshold
        << "\n"
        << "Compute vStar: " << config_.vstar << "\n"
        << "Compute greedy version - VStar - instead of continuous dijkstra: "
        << config_.greedy << "\n"
        << "Compute visibility-based implementation: "
        << config_.visibilityBasedSolver << "\n"
        << "Compute MSFM in the MATLAB interface: " << config_.msfm << "\n"
        << "Expand in obstacles: " << config_.expandInObstacles << "\n"
        << "Compute basic AStar: " << config_.astar << "\n"
        << "Compute distance functions: " << config_.distanceFunction
        << std::endl;
    std::cout << "Initial frontline: ";
    for (size_t i = 0; i < config_.initialFrontline.size(); ++i) {
      std::cout << config_.initialFrontline.at(i);
      if (i < config_.initialFrontline.size() - 1) {
        std::cout << ", ";
      } else {
        std::cout << "\n";
      }
    }
    std::cout << "Target: " << config_.target_x << ", " << config_.target_y
              << std::endl;
    std::cout << "############################ Output settings "
                 "############################## \n"
              << "timer: " << config_.timer << "\n"
              << "saveResults: " << config_.saveResults << "\n"
              << "saveCameFrom: " << config_.saveCameFrom << "\n"
              << "saveGScore: " << config_.saveGScore << "\n"
              << "saveFScore: " << config_.saveFScore << "\n"
              << "saveLightSources: " << config_.saveLightSources << "\n"
              << "saveVisibilityField: " << config_.saveVisibilityField << "\n"
              << "saveSpeedField: " << config_.saveSpeedField << "\n"
              << "savePath: " << config_.savePath << "\n"
              << "saveDistanceFunction: " << config_.saveDistanceFunction
              << std::endl;
  }
  return true;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
std::vector<int> ConfigParser::parseVectorString(const std::string &str) {
  if (str.size() < 2 || str.front() != '{' || str.back() != '}') {
    throw std::invalid_argument("missing braces");
  }

  std::string innerStr = str.substr(1, str.size() - 2);
  std::vector<int> result;
  if (innerStr.find_first_not_of(" \t") == std::string::npos) {
    return result;
  }

  std::istringstream input(innerStr);
  for (std::string token; std::getline(input, token, ',');) {
    const size_t first = token.find_first_not_of(" \t");
    if (first == std::string::npos) {
      throw std::invalid_argument("empty coordinate");
    }
    const size_t last = token.find_last_not_of(" \t");
    token = token.substr(first, last - first + 1);

    size_t parsedCharacters = 0;
    const int coordinate = std::stoi(token, &parsedCharacters);
    if (parsedCharacters != token.size()) {
      throw std::invalid_argument("invalid coordinate");
    }
    result.push_back(coordinate);
  }

  if (!innerStr.empty() && innerStr.back() == ',') {
    throw std::invalid_argument("empty coordinate");
  }
  return result;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
const Config &ConfigParser::getConfig() const { return config_; }

} // namespace vbm
