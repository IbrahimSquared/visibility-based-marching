#include "parser/ConfigParser.h"

#include <fstream>
#include <sstream>
#include <iostream>

namespace vbs {

bool ConfigParser::parse(const std::string& filename) {
  std::ifstream file(filename);
  if (!file) {
    std::cerr << "Failed to open " << filename << '\n';
    return false;
  }
  
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
          std::cerr << "Invalid value for " << key << ": " << value << ", using default value 1\n";
          config_.mode = 1;
        }
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
      }
    } else if (key == "ncols") {
      try {
        config_.ncols = std::stoul(value);
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
      }
    } else if (key == "nrows") {
      try {
        config_.nrows = std::stoul(value);
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
      }
    } else if (key == "nb_of_obstacles") {
      try {
        config_.nb_of_obstacles = std::stoi(value);
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
      }
    } else if (key == "minWidth") {
      try {
        config_.minWidth = std::stoul(value);
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
      }
    } else if (key == "maxWidth") {
      try {
        config_.maxWidth = std::stoul(value);
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
      }
    } else if (key == "minHeight") {
      try {
        config_.minHeight = std::stoul(value);
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
      }
    } else if (key == "maxHeight") {
      try {
        config_.maxHeight = std::stoul(value);
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
      }
    } else if (key == "randomSeed") {
      if (value == "0" || value == "false") {
        config_.randomSeed = false;
      } else if (value == "1" || value == "true") {
        config_.randomSeed = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
      }
    } else if (key == "seedValue") {
      try {
        config_.seedValue = std::stoi(value);
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
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
      }
    } else if (key == "visibilityBasedSolver") {
      if (value == "0" || value == "false") {
        config_.visibilityBasedSolver = false;
      } else if (value == "1" || value == "true") {
        config_.visibilityBasedSolver = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
      }
    } else if (key == "expandInObstacles") {
      if (value == "0" || value == "false") {
        config_.expandInObstacles = false;
      } else if (value == "1" || value == "true") {
        config_.expandInObstacles = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
      }
    } else if (key == "vstar") {
      if (value == "0" || value == "false") {
        config_.vstar = false;
      } else if (value == "1" || value == "true") {
        config_.vstar = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
      }
    } else if (key == "astar") {
      if (value == "0" || value == "false") {
        config_.astar = false;
      } else if (value == "1" || value == "true") {
        config_.astar = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
      }
    } else if (key == "distanceFunction") {
      if (value == "0" || value == "false") {
        config_.distanceFunction = false;
      } else if (value == "1" || value == "true") {
        config_.distanceFunction = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
      }
    } else if (key == "initialFrontline") {
      config_.initialFrontline = parseVectorString(value);
    } else if (key == "target_x") {
      try {
        config_.target_x = std::stoi(value);
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
      }
    } else if (key == "target_y") {
      try {
        config_.target_y = std::stoi(value);
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
      }
    } else if (key == "visibilityThreshold") {
      try {
        config_.visibilityThreshold  = std::stod(value);
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
      }
    } else if (key == "speedValue") {
      try {
        config_.speedValue  = std::stod(value);
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
      }
    } else if (key == "timer") {
      if (value == "0" || value == "false") {
        config_.timer = false;
      } else if (value == "1" || value == "true") {
        config_.timer = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
      }
    } else if (key == "saveResults") {
      if (value == "0" || value == "false") {
        config_.saveResults = false;
      } else if (value == "1" || value == "true") {
        config_.saveResults = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
      }
    } else if (key == "saveCameFrom") {
      if (value == "0" || value == "false") {
        config_.saveCameFrom = false;
      } else if (value == "1" || value == "true") {
        config_.saveCameFrom = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
      }
    } else if (key == "saveGScore") {
      if (value == "0" || value == "false") {
        config_.saveGScore = false;
      } else if (value == "1" || value == "true") {
        config_.saveGScore = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
      }
    } else if (key == "saveLightSources") {
      if (value == "0" || value == "false") {
        config_.saveLightSources = false;
      } else if (value == "1" || value == "true") {
        config_.saveLightSources = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
      }
    } else if (key == "saveVisibilityField") {
      if (value == "0" || value == "false") {
        config_.saveVisibilityField = false;
      } else if (value == "1" || value == "true") {
        config_.saveVisibilityField = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
      }
    } else if (key == "saveSpeedField") {
      if (value == "0" || value == "false") {
        config_.saveSpeedField = false;
      } else if (value == "1" || value == "true") {
        config_.saveSpeedField = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
      }
    } else if (key == "savePath") {
      if (value == "0" || value == "false") {
        config_.savePath = false;
      } else if (value == "1" || value == "true") {
        config_.savePath = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
      }
    } else if (key == "saveFScore") {
      if (value == "0" || value == "false") {
        config_.saveFScore = false;
      } else if (value == "1" || value == "true") {
        config_.saveFScore = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
      }
    } else if (key == "saveDistanceFunction") {
      if (value == "0" || value == "false") {
        config_.saveDistanceFunction = false;
      } else if (value == "1" || value == "true") {
        config_.saveDistanceFunction = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
      }
    } else if (key == "silent") {
      if (value == "0" || value == "false") {
        config_.silent = false;
      } else if (value == "1" || value == "true") {
        config_.silent = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
      }
    }  else {
      std::cerr << "Invalid key: " << key << '\n';
    }
  }
  if (!config_.silent) {
    if (config_.mode == 1) {
      std::cout << "Random environment mode" << std::endl;
      std::cout << "########################### Environment settings ########################## \n"
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
      std::cout << "Import image mode" << "\n"
        << "Image path: " << config_.imagePath << std::endl;
    }
    std::cout << "############################ Solver settings ############################## \n" 
      << "Solver visibility threshold: " << config_.visibilityThreshold << "\n"
      << "Compute vStar: " << config_.vstar << "\n"
      << "Compute greedy version - VStar - instead of continuous dijkstra: " << config_.greedy << "\n"
      << "Compute visibility-based implementation: " << config_.visibilityBasedSolver << "\n"
      << "Expand in obstacles: " << config_.expandInObstacles << "\n"
      << "Compute basic AStar: " << config_.astar << "\n"
      << "Compute distance functions: " << config_.distanceFunction << std::endl;
    std::cout << "Initial frontline: ";
    for (size_t i = 0; i < config_.initialFrontline.size(); ++i) {
      std::cout << config_.initialFrontline.at(i);
      if (i < config_.initialFrontline.size() - 1) {
        std::cout << ", ";
      } else {
        std::cout << "\n";
      }
    }
    std::cout << "Target: " << config_.target_x << ", " << config_.target_y << std::endl;
    std::cout << "############################ Output settings ############################## \n" 
      << "timer: " << config_.timer << "\n"
      << "saveResults: " << config_.saveResults << "\n"
      << "saveCameFrom: " << config_.saveCameFrom << "\n"
      << "saveGScore: " << config_.saveGScore << "\n"
      << "saveFScore: " << config_.saveFScore << "\n"
      << "saveLightSources: " << config_.saveLightSources << "\n"
      << "saveVisibilityField: " << config_.saveVisibilityField << "\n"
      << "saveSpeedField: " << config_.saveSpeedField << "\n"
      << "savePath: " << config_.savePath << "\n"
      << "saveDistanceFunction: " << config_.saveDistanceFunction << std::endl;
  }
  // Return true if we successfully parsed the config file
  return true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<int> ConfigParser::parseVectorString(const std::string& str) {
    // Remove outer braces
    std::string innerStr = str.substr(1, str.size() - 2);
    // Split string into individual integers
    std::vector<int> result;
    std::string delimiter = ",";
    size_t pos = 0;
    while ((pos = innerStr.find(delimiter)) != std::string::npos) {
        std::string token = innerStr.substr(0, pos);
        result.push_back(std::stoi(token));
        innerStr.erase(0, pos + delimiter.length());
    }
    result.push_back(std::stoi(innerStr)); // Add the last integer
    return result;
}

} // namespace vbs