#include "solver/solver.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace vbm {

template <typename T> auto durationInMicroseconds(T start, T end) {
  return std::chrono::duration_cast<std::chrono::microseconds>(end - start)
      .count();
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
sf::Color getColor(double value) {
  // jet colormap for SFML visualization/plot
  const int color_index = 255 * value;
  double r, g, b;
  if (color_index < 32) {
    r = 0;
    g = 0;
    b = 0.5156 + 0.0156 * color_index;
  } else if (color_index < 96) {
    r = 0;
    g = 0.0156 + 0.9844 * (color_index - 32.0) / 64;
    b = 1;
  } else if (color_index < 158) {
    r = 0.0156 + (color_index - 96.0) / 64;
    g = 1;
    b = 0.9844 - (color_index - 96.0) / 64;
  } else if (color_index < 223) {
    r = 1;
    g = 1 - (color_index - 158.0) / 65;
    b = 0;
  } else {
    r = (2 - (color_index - 223.0) / 32) / 2.0;
    g = 0;
    b = 0;
  }
  return sf::Color(static_cast<sf::Uint8>(r * 255),
                   static_cast<sf::Uint8>(g * 255),
                   static_cast<sf::Uint8>(b * 255));
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
Solver::Solver(Environment &env)
    : sharedConfig_(env.getConfig()),
      sharedVisibilityField_(env.getVisibilityField()),
      sharedSpeedField_(env.getSpeedField()) {
  nx_ = sharedVisibilityField_->nx();
  ny_ = sharedVisibilityField_->ny();
  visibilityThreshold_ = sharedConfig_->visibilityThreshold;

  // Init environment image
  uniqueLoadedImage_.reset(std::make_unique<sf::Image>().release());
  uniqueLoadedImage_->create(nx_, ny_, sf::Color::Black);
  sf::Color color;
  color.a = 1;
  for (size_t i = 0; i < nx_; ++i) {
    for (size_t j = 0; j < ny_; ++j) {
      if (sharedVisibilityField_->get(i, j) < 1) {
        uniqueLoadedImage_->setPixel(i, j, color.Black);
      } else {
        uniqueLoadedImage_->setPixel(i, j, color.White);
      }
    }
  }
  // Init maps
  reset();
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Solver::reset() {
  gScore_.reset(nx_, ny_, std::numeric_limits<double>::infinity());
  fScore_.reset(nx_, ny_, std::numeric_limits<double>::infinity());
  cameFrom_.reset(nx_, ny_, 0);
  inOpenSet_.reset(nx_, ny_, false);
  updated_.reset(nx_, ny_, false);

  lightSources_.reset(new point[nx_ * ny_]);

  visibilityHashMap_.clear();
  openSet_.reset();

  // Reserve openSet_
  std::vector<Node> container;
  container.reserve(nx_ * ny_);
  std::priority_queue<Node, std::vector<Node>, std::less<Node>> heap(
      std::less<Node>(), std::move(container));
  openSet_ = std::make_unique<std::priority_queue<Node>>(heap);

  nb_of_sources_ = 0;
  nb_of_iterations_ = 0;

  // Reserve hash map
  visibilityHashMap_.reserve(nx_ * ny_);
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Solver::visibilityBasedSolver() {
  reset();
  auto startTime = std::chrono::high_resolution_clock::now();

  // Init
  double d = 0;
  int x = 0, y = 0, neighbour_x = 0, neighbour_y = 0;

  auto &initial_frontline = sharedConfig_->initialFrontline;

  if (initial_frontline.size() % 2 != 0) {
    std::cout << "###################### Visibility-based solver output "
                 "######################"
              << std::endl;
    std::cout << "Initial frontline must be of size that is a multiple of 2 "
                 "for visibility-based solver"
              << std::endl;
    return;
  }
  for (size_t i = 0; i < initial_frontline.size(); i += 2) {
    x = initial_frontline[i];
    y = ny_ - 1 - initial_frontline[i + 1];
    // check if starting positions are inside the map
    if (x >= nx_ || y >= ny_) {
      std::cout << "###################### Visibility-based solver output "
                   "######################"
                << std::endl;
      std::cout << "At least one of the starting positions is outside the map"
                << std::endl;
      return;
    }

    if (sharedVisibilityField_->get(x, y) < 1) {
      std::cout << "###################### Visibility-based solver output "
                   "######################"
                << std::endl;
      std::cout << "At least one of the starting positions is invalid/occupied"
                << std::endl;
      return;
    }

    d = 0;
    gScore_(x, y) = d;
    updated_(x, y) = true;
    cameFrom_(x, y) = nb_of_sources_;
    lightSources_[nb_of_sources_] = {x, y};

    openSet_->push(Node{x, y, d});
    const auto key = hashFunction(x, y, nb_of_sources_);
    visibilityHashMap_[key] = lightStrength_;
    ++nb_of_sources_;
    ++nb_of_iterations_;
  }

  // For queing unique sources from neighbours of neighbour
  std::vector<size_t> potentialSources;
  potentialSources.reserve(10);
  std::vector<std::pair<double, size_t>> potentialDistances;
  potentialDistances.reserve(10);

  double distance = 0;

  while (openSet_->size() > 0) {
    auto &current = openSet_->top();
    x = current.x;
    y = current.y;
    openSet_->pop();

    // Expand frontline at current & update neighbours
    for (size_t j = 0; j < 16; j += 2) {
      // NOTE those are always positive
      neighbour_x = x + neighbours_[j];
      neighbour_y = y + neighbours_[j + 1];

      // Box check
      if (neighbour_x >= nx_ || neighbour_x < 0 || neighbour_y >= ny_ ||
          neighbour_y < 0) {
        continue;
      };
      if (updated_(neighbour_x, neighbour_y)) {
        continue;
      };
      if (sharedVisibilityField_->get(neighbour_x, neighbour_y) < 1) {
        if (sharedConfig_->expandInObstacles) {
          gScore_(neighbour_x, neighbour_y) =
              gScore_(x, y) +
              evaluateDistanceSpeedField(x, y, neighbour_x, neighbour_y);
          openSet_->push(Node{neighbour_x, neighbour_y,
                              gScore_(neighbour_x, neighbour_y)});
          const auto key = hashFunction(x, y, nb_of_sources_);
          visibilityHashMap_[key] =
              sharedVisibilityField_->get(neighbour_x, neighbour_y);
          ;
        }
        cameFrom_(neighbour_x, neighbour_y) = cameFrom_(x, y);
        updated_(neighbour_x, neighbour_y) = true;
        continue;
      }

      queuePotentialSources(potentialSources, neighbour_x, neighbour_y);
      getPotentialDistancesSpeedField(potentialSources, potentialDistances,
                                      neighbour_x, neighbour_y);

      auto minimum_element =
          std::min_element(potentialDistances.begin(), potentialDistances.end(),
                           [](const auto &lhs, const auto &rhs) {
                             return lhs.first < rhs.first;
                           });
      distance = minimum_element->first;

      if (distance == std::numeric_limits<double>::infinity()) {
        createNewPivot(x, y, neighbour_x, neighbour_y);
      } else {
        // use source giving least distance
        gScore_(neighbour_x, neighbour_y) = minimum_element->first;
        cameFrom_(neighbour_x, neighbour_y) = minimum_element->second;
      }
      openSet_->push(
          Node{neighbour_x, neighbour_y, gScore_(neighbour_x, neighbour_y)});
      updated_(neighbour_x, neighbour_y) = true;
      ++nb_of_iterations_;
    }
  };

  auto stopTime = std::chrono::high_resolution_clock::now();
  auto executionDuration = durationInMicroseconds(startTime, stopTime);

  if (!sharedConfig_->silent) {
    std::cout << "###################### Visibility-based solver output "
                 "######################"
              << std::endl;
    if (sharedConfig_->timer) {
      std::cout << "Execution time in us: " << executionDuration << "us"
                << std::endl;
      std::cout << "Load factor: " << visibilityHashMap_.load_factor()
                << std::endl;
      std::cout << "Iterations: " << nb_of_iterations_ << std::endl;
      std::cout << "Nb of sources: " << nb_of_sources_ << std::endl;
    }
  }
  if (sharedConfig_->saveResults) {
    saveResults({}, "visibilityBased");
  }
  if (sharedConfig_->saveVisibilityBasedSolverImage) {
    saveVisibilityBasedSolverImage(gScore_);
  }
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Solver::vStarSearch() {
  reset();
  auto startTime = std::chrono::high_resolution_clock::now();

  // Init
  double g = 0, h = 0, f = 0;
  int x = 0, y = 0, neighbour_x = 0, neighbour_y = 0;
  int endX = sharedConfig_->target_x;
  int endY = ny_ - 1 - sharedConfig_->target_y;

  if (endX >= nx_ || endY >= ny_) {
    std::cout << "###################### VStar solver output "
                 "######################"
              << std::endl;
    std::cout << "Target position is outside the map" << std::endl;
    return;
  }

  if (sharedVisibilityField_->get(endX, endY) < 1) {
    std::cout << "###################### VStar solver output "
                 "######################"
              << std::endl;
    std::cout << "Target position is invalid/occupied" << std::endl;
    return;
  }

  auto &initial_frontline = sharedConfig_->initialFrontline;
  if (initial_frontline.size() != 2) {
    std::cout << "###################### VStar solver output "
                 "######################"
              << std::endl;
    std::cout << "Initial frontline must be of size 2 for vStar" << std::endl;
    return;
  }

  // Fill in data from initial frontline
  for (size_t i = 0; i < initial_frontline.size(); i += 2) {
    x = initial_frontline[i];
    y = ny_ - 1 - initial_frontline[i + 1];

    if (x >= nx_ || y >= ny_) {
      std::cout << "The starting position is outside the map" << std::endl;
      return;
    }

    if (sharedVisibilityField_->get(x, y) < 1) {
      std::cout << "Starting position is invalid/occupied" << std::endl;
      return;
    }
    g = 0;
    h = 0;
    if (sharedConfig_->greedy) {
      h = evaluateDistance(x, y, endX, endY);
    }
    f = g + h;
    openSet_->push(Node{x, y, f});

    gScore_(x, y) = g;
    fScore_(x, y) = f;
    cameFrom_(x, y) = nb_of_sources_;
    updated_(x, y) = true;
    lightSources_[nb_of_sources_] = {x, y};
    const auto key = hashFunction(x, y, nb_of_sources_);
    visibilityHashMap_[key] = lightStrength_;
    ++nb_of_sources_;
    ++nb_of_iterations_;
  }

  // For queing unique sources from neighbours of neighbour
  std::vector<size_t> potentialSources;
  potentialSources.reserve(10);
  std::vector<std::pair<double, size_t>> potentialDistances;
  potentialDistances.reserve(10);

  double distance = 0;

  while (openSet_->size() > 0) {
    auto &current = openSet_->top();
    x = current.x;
    y = current.y;
    if (x == endX && y == endY) {
      auto stopTime = std::chrono::high_resolution_clock::now();
      auto executionDuration = durationInMicroseconds(startTime, stopTime);

      if (sharedConfig_->silent) {
        return;
      }
      std::cout << "###################### VStar solver output "
                   "######################"
                << std::endl;
      std::cout << "Path found" << std::endl;
      if (!sharedConfig_->timer) {
        return;
      }
      std::cout << "Execution time in us: " << executionDuration << "us"
                << std::endl;
      std::cout << "Load factor: " << visibilityHashMap_.load_factor()
                << std::endl;
      std::cout << "Iterations: " << nb_of_iterations_ << std::endl;
      reconstructPath(current, "vstar");
      return;
    }

    openSet_->pop();

    // Expand frontline at current & update neighbours
    for (size_t j = 0; j < 16; j += 2) {
      // NOTE those are always positive
      neighbour_x = x + neighbours_[j];
      neighbour_y = y + neighbours_[j + 1];

      // Box check
      if (neighbour_x >= nx_ || neighbour_x < 0 || neighbour_y >= ny_ ||
          neighbour_y < 0) {
        continue;
      };
      if (updated_(neighbour_x, neighbour_y)) {
        continue;
      };
      if (sharedVisibilityField_->get(neighbour_x, neighbour_y) < 1) {
        cameFrom_(neighbour_x, neighbour_y) = cameFrom_(x, y);
        updated_(neighbour_x, neighbour_y) = true;
        continue;
      }

      queuePotentialSources(potentialSources, neighbour_x, neighbour_y);
      getPotentialDistances(potentialSources, potentialDistances, neighbour_x,
                            neighbour_y);

      auto minimum_element =
          std::min_element(potentialDistances.begin(), potentialDistances.end(),
                           [](const auto &lhs, const auto &rhs) {
                             return lhs.first < rhs.first;
                           });
      distance = minimum_element->first;

      if (distance == std::numeric_limits<double>::infinity()) {
        createNewPivot(x, y, neighbour_x, neighbour_y);
      } else {
        // use source giving least distance
        gScore_(neighbour_x, neighbour_y) = minimum_element->first;
        cameFrom_(neighbour_x, neighbour_y) = minimum_element->second;
      }

      h = 0;
      if (sharedConfig_->greedy) {
        h = evaluateDistance(neighbour_x, neighbour_y, endX, endY);
      }
      f = gScore_(neighbour_x, neighbour_y) + h;
      fScore_(neighbour_x, neighbour_y) = f;

      openSet_->push(Node{neighbour_x, neighbour_y, f});
      updated_(neighbour_x, neighbour_y) = true;
      ++nb_of_iterations_;
    }
  }

  auto stopTime = std::chrono::high_resolution_clock::now();
  auto executionDuration = durationInMicroseconds(startTime, stopTime);

  if (!sharedConfig_->silent) {
    std::cout << "###################### VStar solver output "
                 "######################"
              << std::endl;
    if (sharedConfig_->timer) {
      std::cout << "Path could not be found" << std::endl;
      std::cout << "Execution time in us: " << executionDuration << "us"
                << std::endl;
      std::cout << "Load factor: " << visibilityHashMap_.load_factor()
                << std::endl;
      std::cout << "Iterations: " << nb_of_iterations_ << std::endl;
      reconstructPath({}, "vstar");
    }
  }
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Solver::aStarSearch() {
  reset();
  const auto startTime = std::chrono::high_resolution_clock::now();

  // Init
  double g = 0, h = 0, f = 0;
  int x = 0, y = 0, neighbour_x = 0, neighbour_y = 0;
  const int endX = sharedConfig_->target_x;
  const int endY = ny_ - 1 - sharedConfig_->target_y;

  // check if target is inside the map
  if (endX >= nx_ || endY >= ny_) {
    std::cout << "###################### AStar solver output "
                 "######################"
              << std::endl;
    std::cout << "Target position is outside the map" << std::endl;
    return;
  }

  // check if target is feasible
  if (sharedVisibilityField_->get(endX, endY) < 1) {
    std::cout << "###################### AStar solver output "
                 "######################"
              << std::endl;

    std::cout << "Target position is invalid/occupied" << std::endl;
    return;
  }

  auto &initial_frontline = sharedConfig_->initialFrontline;

  // To use astar, initial frontline must be of size 2
  if (initial_frontline.size() != 2) {
    std::cout << "##################### AStar solver output "
                 "#####################"
              << std::endl;

    std::cout << "Initial frontline must be of size 2 for aStar" << std::endl;
    return;
  }

  // Fill in data from initial frontline
  for (size_t i = 0; i < initial_frontline.size(); i += 2) {
    x = initial_frontline[i];
    y = ny_ - 1 - initial_frontline[i + 1];

    // check if starting positions are inside the map
    if (x >= nx_ || y >= ny_) {
      std::cout << "The starting position is outside the map" << std::endl;
      return;
    }

    // Check if starting position is valid
    if (sharedVisibilityField_->get(x, y) < 1) {
      std::cout << "Starting position is invalid/occupied" << std::endl;
      return;
    }

    g = 0;
    h = 0;
    if (sharedConfig_->greedy) {
      h = evaluateDistance(x, y, endX, endY);
    }
    f = g + h;
    openSet_->push(Node{x, y, f});
    inOpenSet_(x, y) = true;

    gScore_(x, y) = g;
    fScore_(x, y) = f;
    cameFrom_(x, y) = indexAt(x, y);
    ++nb_of_iterations_;
  }

  while (openSet_->size() > 0) {
    auto &current = openSet_->top();
    x = current.x;
    y = current.y;

    if (x == endX && y == endY) {
      const auto stopTime = std::chrono::high_resolution_clock::now();
      const auto executionDuration =
          durationInMicroseconds(startTime, stopTime);

      if (sharedConfig_->silent) {
        return;
      }
      std::cout << "##################### AStar solver output "
                   "#####################"
                << std::endl;
      std::cout << "Path found" << std::endl;
      if (!sharedConfig_->timer) {
        return;
      }
      std::cout << "Execution time in us: " << executionDuration << "us"
                << std::endl;
      std::cout << "Iterations: " << nb_of_iterations_ << std::endl;
      reconstructPath(current, "astar");
      return;
    }

    openSet_->pop();
    inOpenSet_(x, y) = false;

    // Expand frontline at current & update neighbours
    for (size_t j = 0; j < 16; j += 2) {
      // NOTE those are always positive
      neighbour_x = x + neighbours_[j];
      neighbour_y = y + neighbours_[j + 1];

      // Box check
      if (neighbour_x >= nx_ || neighbour_y >= ny_ || neighbour_x < 0 ||
          neighbour_y < 0) {
        continue;
      };
      if (sharedVisibilityField_->get(neighbour_x, neighbour_y) < 1) {
        continue;
      }

      g = gScore_(x, y) + evaluateDistance(x, y, neighbour_x, neighbour_y);
      if (g >= gScore_(neighbour_x, neighbour_y)) {
        continue;
      };

      cameFrom_(neighbour_x, neighbour_y) = indexAt(x, y);
      gScore_(neighbour_x, neighbour_y) = g;
      ;
      if (sharedConfig_->greedy) {
        h = evaluateDistance(neighbour_x, neighbour_y, endX, endY);
      }
      f = gScore_(neighbour_x, neighbour_y) + h;
      fScore_(neighbour_x, neighbour_y) = f;
      ++nb_of_iterations_;
      if (inOpenSet_(neighbour_x, neighbour_y)) {
        continue;
      }

      openSet_->push(Node{neighbour_x, neighbour_y, f});
      inOpenSet_(neighbour_x, neighbour_y) = true;
      ;
    }
  };

  auto stopTime = std::chrono::high_resolution_clock::now();
  auto executionDuration = durationInMicroseconds(startTime, stopTime);

  if (!sharedConfig_->silent) {
    std::cout << "##################### AStar Solver output "
                 "#####################"
              << std::endl;
    if (sharedConfig_->timer) {
      std::cout << "Path could not be found" << std::endl;
      std::cout << "Execution time in us: " << executionDuration << "us"
                << std::endl;
      std::cout << "Iterations: " << nb_of_iterations_ << std::endl;
      reconstructPath({}, "astar");
    }
  }
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Solver::computeDistanceFunction() {
  reset();
  auto startTime = std::chrono::high_resolution_clock::now();
  std::vector<int> initial_frontline;
  for (size_t i = 0; i < nx_; ++i) {
    for (size_t j = 0; j < ny_; ++j) {
      if (sharedVisibilityField_->get(i, j) <= visibilityThreshold_) {
        initial_frontline.push_back(i);
        initial_frontline.push_back(j);
      }
    }
  }
  // Init
  int x = 0, y = 0, neighbour_x = 0, neighbour_y = 0;
  double g = 0;
  // Fill in data from initial frontline
  for (size_t i = 0; i < initial_frontline.size(); i += 2) {
    x = initial_frontline[i];
    y = initial_frontline[i + 1];
    g = 0;
    openSet_->push(Node{x, y, g});

    gScore_(x, y) = g;
    updated_(x, y) = true;
    cameFrom_(x, y) = nb_of_sources_;
    lightSources_[nb_of_sources_] = {x, y};
    const auto key = hashFunction(x, y, nb_of_sources_);
    visibilityHashMap_[key] = lightStrength_;
    ++nb_of_sources_;
  }

  // For queing unique sources from neighbours of neighbour
  std::vector<size_t> potentialSources;
  potentialSources.reserve(10);
  std::vector<std::pair<double, size_t>> potentialDistances;
  potentialDistances.reserve(10);

  double distance = 0;

  while (openSet_->size() > 0) {
    auto &current = openSet_->top();
    x = current.x;
    y = current.y;

    openSet_->pop();

    // Expand frontline at current & update neighbours
    for (size_t j = 0; j < 16; j += 2) {
      // NOTE those are always positive
      neighbour_x = x + neighbours_[j];
      neighbour_y = y + neighbours_[j + 1];

      // Box check
      if (neighbour_x >= nx_ || neighbour_x < 0 || neighbour_y >= ny_ ||
          neighbour_y < 0) {
        continue;
      };
      if (updated_(neighbour_x, neighbour_y)) {
        continue;
      };

      queuePotentialSources(potentialSources, neighbour_x, neighbour_y);
      potentialDistances.clear();
      for (int k = 0; k < potentialSources.size(); ++k) {
        int potentialSource = potentialSources[k];
        int LS_x = lightSources_[potentialSource].x;
        int LS_y = lightSources_[potentialSource].y;
        distance = gScore_(LS_x, LS_y) +
                   evaluateDistance(LS_x, LS_y, neighbour_x, neighbour_y);
        potentialDistances.push_back(
            std::pair<double, int>{distance, potentialSource});
      }

      auto minimum_element =
          std::min_element(potentialDistances.begin(), potentialDistances.end(),
                           [](const auto &lhs, const auto &rhs) {
                             return lhs.first < rhs.first;
                           });
      distance = minimum_element->first;
      // use source giving least distance
      gScore_(neighbour_x, neighbour_y) = distance;
      cameFrom_(neighbour_x, neighbour_y) = minimum_element->second;
      openSet_->push(Node{neighbour_x, neighbour_y, distance});
      updated_(neighbour_x, neighbour_y) = true;
    }
  };

  auto stopTime = std::chrono::high_resolution_clock::now();
  auto executionDuration = durationInMicroseconds(startTime, stopTime);

  if (!sharedConfig_->silent) {
    std::cout << "##################### Distance function computation "
                 "output #####################"
              << std::endl;
    if (sharedConfig_->timer) {
      std::cout << "Constructed distance function" << std::endl;
      std::cout << "Execution time in us: " << executionDuration << "us"
                << std::endl;
    }
  }
  if (sharedConfig_->saveResults) {
    saveResults({}, "distanceFunction");
  }
  if (sharedConfig_->saveDistanceFunctionImage) {
    saveDistanceFunctionImage(gScore_);
  }
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
inline void Solver::queuePotentialSources(std::vector<size_t> &potentialSources,
                                          const int neighbour_x,
                                          const int neighbour_y) const {
  size_t potentialSource_x = 0, potentialSource_y = 0, lightSource_num = 0;
  potentialSources.clear();
  // Queue sources from updated neighbours of neighbour
  for (size_t k = 0; k < 16; k += 2) {
    // NOTE those are always positive
    potentialSource_x = neighbour_x + neighbours_[k];
    potentialSource_y = neighbour_y + neighbours_[k + 1];
    // Box check
    if (potentialSource_x >= nx_ || potentialSource_y >= ny_) {
      continue;
    };
    if (!updated_(potentialSource_x, potentialSource_y)) {
      continue;
    };

    lightSource_num = cameFrom_(potentialSource_x, potentialSource_y);
    // Pick only unique sources (no repitition in potentialSources)
    if (std::find(potentialSources.begin(), potentialSources.end(),
                  lightSource_num) == potentialSources.end()) {
      potentialSources.push_back(lightSource_num);
    }
  }
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Solver::getPotentialDistances(
    const std::vector<size_t> &potentialSources,
    std::vector<std::pair<double, size_t>> &potentialDistances,
    const int neighbour_x, const int neighbour_y) {
  size_t LS_x = 0, LS_y = 0, potentialSource = 0;
  double distance = 0;
  potentialDistances.clear();
  for (size_t k = 0; k < potentialSources.size(); ++k) {
    potentialSource = potentialSources[k];
    LS_x = lightSources_[potentialSource].x;
    LS_y = lightSources_[potentialSource].y;
    // update visibility from source
    updatePointVisibility(potentialSource, LS_x, LS_y, neighbour_x,
                          neighbour_y);
    distance = INFINITY;
    const auto key = hashFunction(neighbour_x, neighbour_y, potentialSource);
    if (visibilityHashMap_.at(key) >= visibilityThreshold_) {
      distance = gScore_(LS_x, LS_y) +
                 evaluateDistance(LS_x, LS_y, neighbour_x, neighbour_y);
    }
    potentialDistances.push_back(
        std::pair<double, int>{distance, potentialSource});
  }
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Solver::getPotentialDistancesSpeedField(
    const std::vector<size_t> &potentialSources,
    std::vector<std::pair<double, size_t>> &potentialDistances,
    const int neighbour_x, const int neighbour_y) {
  size_t LS_x = 0, LS_y = 0, potentialSource = 0;
  double distance = 0;
  potentialDistances.clear();
  for (size_t k = 0; k < potentialSources.size(); ++k) {
    potentialSource = potentialSources[k];
    LS_x = lightSources_[potentialSource].x;
    LS_y = lightSources_[potentialSource].y;
    // update visibility from source
    updatePointVisibility(potentialSource, LS_x, LS_y, neighbour_x,
                          neighbour_y);
    distance = INFINITY;
    const auto key = hashFunction(neighbour_x, neighbour_y, potentialSource);
    if (visibilityHashMap_.at(key) >= visibilityThreshold_) {
      distance =
          gScore_(LS_x, LS_y) +
          evaluateDistanceSpeedField(LS_x, LS_y, neighbour_x, neighbour_y);
    }
    potentialDistances.push_back(
        std::pair<double, int>{distance, potentialSource});
  }
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Solver::createNewPivot(const int x, const int y, const int neighbour_x,
                            const int neighbour_y) {
  int pivot_neighbour_x, pivot_neighbour_y;
  // Pushback parent as a new lightSource
  lightSources_[nb_of_sources_] = {x, y}; // {x, y};
  // Pusback pivot & update light source visibility
  const auto key = hashFunction(x, y, nb_of_sources_);
  visibilityHashMap_[key] = lightStrength_;
  // Update maps of new pivot_
  // Update neighbours of initial frontline points - both distance & visibility
  for (size_t p = 0; p < 16; p += 2) {
    // NOTE those are always positive
    pivot_neighbour_x = x + neighbours_[p];
    pivot_neighbour_y = y + neighbours_[p + 1];
    // Box check
    if (pivot_neighbour_x >= nx_ || pivot_neighbour_x < 0 ||
        pivot_neighbour_y >= ny_ || pivot_neighbour_y < 0) {
      continue;
    }
    // Update neighbour visibility
    updatePointVisibility(nb_of_sources_, x, y, pivot_neighbour_x,
                          pivot_neighbour_y);
  }
  cameFrom_(neighbour_x, neighbour_y) = nb_of_sources_;
  gScore_(neighbour_x, neighbour_y) =
      gScore_(x, y) + evaluateDistance(x, y, neighbour_x, neighbour_y);
  ++nb_of_sources_;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Solver::updatePointVisibility(const size_t lightSourceNumber,
                                   const int LS_x, const int LS_y, const int x,
                                   const int y) {
  // Variable initialization
  double v = 0;
  double c = 0;

  // Check if visibility value already exists
  auto key = hashFunction(x, y, lightSourceNumber);
  if (visibilityHashMap_.count(key)) {
    return;
  }
  if (sharedVisibilityField_->get(x, y) < visibilityThreshold_) {
    visibilityHashMap_[key] = 0;
    return;
  }

  if (x == LS_x) {
    if (y - LS_y > 0) {
      key = hashFunction(x, y - 1, lightSourceNumber);
      if (!visibilityHashMap_.count(key)) {
        updatePointVisibility(lightSourceNumber, LS_x, LS_y, x, y - 1);
      }
      v = visibilityHashMap_.at(key);
    } else {
      key = hashFunction(x, y + 1, lightSourceNumber);
      if (!visibilityHashMap_.count(key)) {
        updatePointVisibility(lightSourceNumber, LS_x, LS_y, x, y + 1);
      }
      v = visibilityHashMap_.at(key);
    }
  } else if (y == LS_y) {
    if (x - LS_x > 0) {
      key = hashFunction(x - 1, y, lightSourceNumber);
      if (!visibilityHashMap_.count(key)) {
        updatePointVisibility(lightSourceNumber, LS_x, LS_y, x - 1, y);
      }
      v = visibilityHashMap_.at(key);
    } else {
      key = hashFunction(x + 1, y, lightSourceNumber);
      if (!visibilityHashMap_.count(key)) {
        updatePointVisibility(lightSourceNumber, LS_x, LS_y, x + 1, y);
      }
      v = visibilityHashMap_.at(key);
    }
  } else {
    // Q1
    if ((x - LS_x > 0) && (y - LS_y > 0)) {
      key = hashFunction(x - 1, y - 1, lightSourceNumber);
      if (!visibilityHashMap_.count(key)) {
        updatePointVisibility(lightSourceNumber, LS_x, LS_y, x - 1, y - 1);
      }
      if (x - LS_x == y - LS_y) {
        v = visibilityHashMap_.at(key);
      } else if (x - LS_x < y - LS_y) {
        const auto key_1 = hashFunction(x, y - 1, lightSourceNumber);
        if (!visibilityHashMap_.count(key_1)) {
          updatePointVisibility(lightSourceNumber, LS_x, LS_y, x, y - 1);
        }
        c = static_cast<double>(x - LS_x) / (y - LS_y);
        double v1 = visibilityHashMap_.at(key_1);
        v = v1 - c * (v1 - visibilityHashMap_.at(key));
      } else if (x - LS_x > y - LS_y) {
        const auto key_2 = hashFunction(x - 1, y, lightSourceNumber);
        if (!visibilityHashMap_.count(key_2)) {
          updatePointVisibility(lightSourceNumber, LS_x, LS_y, x - 1, y);
        }
        c = static_cast<double>(y - LS_y) / (x - LS_x);
        double v2 = visibilityHashMap_.at(key_2);
        v = v2 - c * (v2 - visibilityHashMap_.at(key));
      }
    }
    // Q2
    else if ((x - LS_x < 0) && (y - LS_y > 0)) {
      key = hashFunction(x + 1, y - 1, lightSourceNumber);
      if (!visibilityHashMap_.count(key)) {
        updatePointVisibility(lightSourceNumber, LS_x, LS_y, x + 1, y - 1);
      }
      if (LS_x - x == y - LS_y) {
        v = visibilityHashMap_.at(key);
      } else if (LS_x - x < y - LS_y) {
        const auto key_1 = hashFunction(x, y - 1, lightSourceNumber);
        if (!visibilityHashMap_.count(key_1)) {
          updatePointVisibility(lightSourceNumber, LS_x, LS_y, x, y - 1);
        }
        c = static_cast<double>(LS_x - x) / (y - LS_y);
        double v1 = visibilityHashMap_.at(key_1);
        v = v1 - c * (v1 - visibilityHashMap_.at(key));
      } else if (LS_x - x > y - LS_y) {
        const auto key_2 = hashFunction(x + 1, y, lightSourceNumber);
        if (!visibilityHashMap_.count(key_2)) {
          updatePointVisibility(lightSourceNumber, LS_x, LS_y, x + 1, y);
        }
        c = static_cast<double>(y - LS_y) / (LS_x - x);
        double v2 = visibilityHashMap_.at(key_2);
        v = v2 - c * (v2 - visibilityHashMap_.at(key));
      }
    }
    // Q3
    else if ((x - LS_x < 0) && (y - LS_y < 0)) {
      key = hashFunction(x + 1, y + 1, lightSourceNumber);
      if (!visibilityHashMap_.count(key)) {
        updatePointVisibility(lightSourceNumber, LS_x, LS_y, x + 1, y + 1);
      }
      if (LS_x - x == LS_y - y) {
        v = visibilityHashMap_.at(key);
      } else if (LS_x - x < LS_y - y) {
        const auto key_1 = (y + 1) + nx_ * x + ny_ * nx_ * lightSourceNumber;
        if (!visibilityHashMap_.count(key_1)) {
          updatePointVisibility(lightSourceNumber, LS_x, LS_y, x, y + 1);
        }
        c = static_cast<double>(LS_x - x) / (LS_y - y);
        double v1 = visibilityHashMap_.at(key_1);
        v = v1 - c * (v1 - visibilityHashMap_.at(key));
      } else if (LS_x - x > LS_y - y) {
        const auto key_2 = hashFunction(x + 1, y, lightSourceNumber);
        if (!visibilityHashMap_.count(key_2)) {
          updatePointVisibility(lightSourceNumber, LS_x, LS_y, x + 1, y);
        }
        c = static_cast<double>(LS_y - y) / (LS_x - x);
        double v2 = visibilityHashMap_.at(key_2);
        v = v2 - c * (v2 - visibilityHashMap_.at(key));
      }
    }
    // Q4
    else if ((x - LS_x > 0) && (y - LS_y < 0)) {
      key = hashFunction(x - 1, y + 1, lightSourceNumber);
      if (!visibilityHashMap_.count(key)) {
        updatePointVisibility(lightSourceNumber, LS_x, LS_y, x - 1, y + 1);
      }
      if (x - LS_x == LS_y - y) {
        v = visibilityHashMap_.at(key);
      } else if (x - LS_x < LS_y - y) {
        const auto key_1 = hashFunction(x, y + 1, lightSourceNumber);
        if (!visibilityHashMap_.count(key_1)) {
          updatePointVisibility(lightSourceNumber, LS_x, LS_y, x, y + 1);
        }
        c = static_cast<double>(x - LS_x) / (LS_y - y);
        double v1 = visibilityHashMap_.at(key_1);
        v = v1 - c * (v1 - visibilityHashMap_.at(key));
      } else if (x - LS_x > LS_y - y) {
        const auto key_2 = hashFunction(x - 1, y, lightSourceNumber);
        if (!visibilityHashMap_.count(key_2)) {
          updatePointVisibility(lightSourceNumber, LS_x, LS_y, x - 1, y);
        }
        c = static_cast<double>(LS_y - y) / (x - LS_x);
        double v2 = visibilityHashMap_.at(key_2);
        v = v2 - c * (v2 - visibilityHashMap_.at(key));
      }
    }
  }
  v = v * sharedVisibilityField_->get(x, y);
  key = hashFunction(x, y, lightSourceNumber);
  visibilityHashMap_[key] = v;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Solver::reconstructPath(const Node &current,
                             const std::string &methodName) {
  std::vector<point> resultingPath;
  int x = current.x, y = current.y;
  double t = cameFrom_(x, y);
  double t_old = INFINITY;
  while (t != t_old) {
    resultingPath.push_back({x, y});
    t_old = t;
    if (methodName == "vstar") {
      x = lightSources_[t].x;
      y = lightSources_[t].y;
    } else if (methodName == "astar") {
      auto p = coordinatesAt(t);
      x = p.x;
      y = p.y;
    }
    t = cameFrom_(x, y);
  }
  resultingPath.push_back({x, y});
  std::reverse(resultingPath.begin(), resultingPath.end());

  if (sharedConfig_->saveResults) {
    saveImageWithPath(resultingPath, methodName);
    saveResults(resultingPath, methodName);
  }

  // compute total distance
  double totalDistance = 0;
  for (size_t i = 0; i < resultingPath.size() - 1; ++i) {
    totalDistance +=
        evaluateDistance(resultingPath[i].x, resultingPath[i].y,
                         resultingPath[i + 1].x, resultingPath[i + 1].y);
  }
  if (!sharedConfig_->silent) {
    std::cout << methodName << " path length: " << totalDistance << std::endl;
  }

  return;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Solver::saveImageWithPath(const std::vector<point> &path,
                               const std::string &methodName) const {
  sf::Image image;
  image = *uniqueLoadedImage_;
  sf::Color color;
  color.a = 1;
  int x0, y0, x1, y1;
  int dx, dy, sx, sy, err;

  for (size_t i = 0; i < path.size() - 1; ++i) {
    x0 = path[i].x;
    y0 = path[i].y;
    x1 = path[i + 1].x;
    y1 = path[i + 1].y;

    dx = std::abs(x1 - x0);
    dy = std::abs(y1 - y0);
    sx = (x0 < x1) ? 1 : -1;
    sy = (y0 < y1) ? 1 : -1;
    err = dx - dy;

    while (x0 != x1 || y0 != y1) {
      image.setPixel(x0, y0, color.Magenta);
      int e2 = 2 * err;
      if (e2 > -dy) {
        err -= dy;
        x0 += sx;
      }
      if (e2 < dx) {
        err += dx;
        y0 += sy;
      }
    }
  }
  int radius = sharedConfig_->ballRadius;
  int x = sharedConfig_->initialFrontline[0];
  int y = ny_ - 1 - sharedConfig_->initialFrontline[1];
  for (int i = -radius; i <= radius; ++i) {
    for (int j = -radius; j <= radius; ++j) {
      if (i * i + j * j <= radius * radius) {
        if (x + i >= 0 && x + i < nx_ && y + j >= 0 && y + j < ny_) {
          image.setPixel(x + i, y + j, color.Green);
        }
      }
    }
  }
  x = sharedConfig_->target_x;
  y = ny_ - 1 - sharedConfig_->target_y;
  for (int i = -radius; i <= radius; ++i) {
    for (int j = -radius; j <= radius; ++j) {
      if (i * i + j * j <= radius * radius) {
        if (x + i >= 0 && x + i < nx_ && y + j >= 0 && y + j < ny_) {
          image.setPixel(x + i, y + j, color.Red);
        }
      }
    }
  }
  std::string imageName = "output/" + methodName + ".png";
  image.saveToFile(imageName);
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Solver::saveVisibilityBasedSolverImage(const Field<double> &gScore) const {
  const int width = nx_;
  const int height = ny_;

  sf::Image image;
  image.create(width, height);

  double minVal = std::numeric_limits<double>::max();
  double maxVal = std::numeric_limits<double>::min();

  // Find min and max values in gScore for normalization
  for (int i = 0; i < width; ++i) {
    for (int j = 0; j < height; ++j) {
      double val = gScore(i, j);
      if (val == std::numeric_limits<double>::infinity())
        continue;
      if (val < minVal)
        minVal = val;
      if (val > maxVal)
        maxVal = val;
    }
  }

  // Generate the image
  for (int i = 0; i < width; ++i) {
    for (int j = 0; j < height; ++j) {
      if (sharedVisibilityField_->get(i, j) < 1) {
        image.setPixel(i, j, sf::Color::Black);
      } else {
        const double normalized_value = gScore(i, j) / (maxVal - minVal);
        sf::Color color = getColor(normalized_value);
        image.setPixel(i, j, color);
      }
    }
  }

  // color all initial frontline points as green circles with radius 10
  sf::Color color;
  color.a = 1;
  int x0, y0;
  int radius = 10;
  for (size_t k = 0; k < sharedConfig_->initialFrontline.size(); k += 2) {
    x0 = sharedConfig_->initialFrontline[k];
    y0 = ny_ - 1 - sharedConfig_->initialFrontline[k + 1];
    for (int i = -radius; i <= radius; ++i) {
      for (int j = -radius; j <= radius; ++j) {
        if (i * i + j * j <= radius * radius) {
          if (x0 + i >= 0 && x0 + i < nx_ && y0 + j >= 0 && y0 + j < ny_) {
            image.setPixel(x0 + i, y0 + j, color.Green);
          }
        }
      }
    }
  }

  // compute the step size based on the max and min values
  int number_of_contour_lines = sharedConfig_->number_of_contour_lines;
  double stepSize = (maxVal - minVal) / number_of_contour_lines;

  std::vector<double> contourLevels;
  for (double level = minVal; level <= maxVal; level += stepSize) {
    contourLevels.push_back(level);
  }
  // Draw contour lines on the image
  for (double level : contourLevels) {
    for (int i = 0; i < width; ++i) {
      for (int j = 0; j < height; ++j) {
        double value = gScore(i, j);
        if (std::abs(value - level) <= stepSize / 15) {
          image.setPixel(i, j, sf::Color::Black);
        }
      }
    }
  }

  std::string outputPath = "./output/visibilityBasedSolver.png";
  image.saveToFile(outputPath);
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Solver::saveDistanceFunctionImage(const Field<double> &gScore) const {
  const int width = nx_;
  const int height = ny_;

  sf::Image image;
  image.create(width, height);

  double minVal = std::numeric_limits<double>::max();
  double maxVal = std::numeric_limits<double>::min();

  // Find min and max values in gScore for normalization
  for (int i = 0; i < width; ++i) {
    for (int j = 0; j < height; ++j) {
      double val = gScore(i, j);
      if (val == std::numeric_limits<double>::infinity())
        continue;
      if (val < minVal)
        minVal = val;
      if (val > maxVal)
        maxVal = val;
    }
  }

  // Generate the image
  for (int i = 0; i < width; ++i) {
    for (int j = 0; j < height; ++j) {
      if (sharedVisibilityField_->get(i, j) < 1) {
        image.setPixel(i, j, sf::Color::Black);
      } else {
        const double normalized_value = gScore(i, j) / (maxVal - minVal);
        sf::Color color = getColor(normalized_value);
        image.setPixel(i, j, color);
      }
    }
  }

  // compute the step size based on the max and min values
  int number_of_contour_lines = sharedConfig_->number_of_contour_lines / 3;
  double stepSize = (maxVal - minVal) / number_of_contour_lines;

  std::vector<double> contourLevels;
  for (double level = minVal; level <= maxVal; level += stepSize) {
    contourLevels.push_back(level);
  }

  // Draw contour lines on the image
  for (double level : contourLevels) {
    for (int i = 0; i < width; ++i) {
      for (int j = 0; j < height; ++j) {
        double value = gScore(i, j);
        if (std::abs(value - level) <= stepSize / 15) {
          image.setPixel(i, j, sf::Color::Black);
        }
      }
    }
  }

  std::string outputPath = "./output/distanceFunction.png";
  image.saveToFile(outputPath);
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Solver::saveResults(const std::vector<point> &resultingPath,
                         const std::string &methodName) const {
  namespace fs = std::filesystem;

  // Define the path to the output file
  std::string outputFilePath = "./output/" + methodName + ".txt";

  // Check if the directory exists, and create it if it doesn't
  fs::path directory = fs::path(outputFilePath).parent_path();
  if (!fs::exists(directory)) {
    if (!fs::create_directories(directory)) {
      std::cerr << "Failed to create directory " << directory.string()
                << std::endl;
      return;
    }
  }

  if (methodName == "distanceFunction") {
    // save distance function
    if (sharedConfig_->saveDistanceFunction) {
      std::fstream of(outputFilePath, std::ios::out | std::ios::trunc);
      if (!of.is_open()) {
        std::cerr << "Failed to open output file " << outputFilePath
                  << std::endl;
        return;
      }
      std::ostream &os = of;
      for (int j = ny_ - 1; j >= 0; --j) {
        for (size_t i = 0; i < nx_; ++i) {
          os << gScore_(i, j) << " ";
        }
        os << "\n";
      }
      of.close();
      if (!sharedConfig_->silent) {
        std::cout << "Saved " + methodName << std::endl;
      }
    }
    return;
  }

  if (methodName == "visibilityBased") {
    std::fstream of(outputFilePath, std::ios::out | std::ios::trunc);
    if (!of.is_open()) {
      std::cerr << "Failed to open output file " << outputFilePath << std::endl;
      return;
    }
    std::ostream &os = of;
    for (int j = ny_ - 1; j >= 0; --j) {
      for (size_t i = 0; i < nx_; ++i) {
        os << gScore_(i, j) << " ";
      }
      os << "\n";
    }
    of.close();
    if (!sharedConfig_->silent) {
      std::cout << "Saved " + methodName + " solution" << std::endl;
    }

    if (sharedConfig_->saveCameFrom) {
      outputFilePath = "./output/" + methodName + "_cameFrom.txt";
      std::fstream of1(outputFilePath, std::ios::out | std::ios::trunc);
      if (!of1.is_open()) {
        std::cerr << "Failed to open output file " << outputFilePath
                  << std::endl;
        return;
      }
      std::ostream &os1 = of1;
      for (int j = ny_ - 1; j >= 0; --j) {
        for (size_t i = 0; i < nx_; ++i) {
          os1 << cameFrom_(i, j) << " ";
        }
        os1 << "\n";
      }
      of1.close();
      if (!sharedConfig_->silent) {
        std::cout << "Saved " + methodName + " cameFrom_" << std::endl;
      }
    }

    if (sharedConfig_->saveLightSources) {
      outputFilePath = "./output/" + methodName + "_lightSources.txt";
      std::fstream of3(outputFilePath, std::ios::out | std::ios::trunc);
      if (!of3.is_open()) {
        std::cerr << "Failed to open output file " << outputFilePath
                  << std::endl;
        return;
      }
      std::ostream &os = of3;
      for (size_t i = 0; i < nb_of_sources_; ++i) {
        os << lightSources_[i].x << " " << ny_ - 1 - lightSources_[i].y;
        os << "\n";
      }
      of3.close();
      if (!sharedConfig_->silent) {
        std::cout << "Saved " + methodName + " lightSources" << std::endl;
      }
    }
    return;
  }

  // Save resulting path
  if (sharedConfig_->savePath) {
    outputFilePath = "./output/" + methodName + "_path.txt";
    std::fstream of(outputFilePath, std::ios::out | std::ios::trunc);
    if (!of.is_open()) {
      std::cerr << "Failed to open output file " << outputFilePath << std::endl;
      return;
    }
    std::ostream &os = of;
    for (size_t i = 0; i < resultingPath.size(); ++i) {
      os << resultingPath[i].x << " " << ny_ - 1 - resultingPath[i].y;
      os << "\n";
    }
    of.close();
    if (!sharedConfig_->silent) {
      std::cout << "Saved " + methodName + " path" << std::endl;
    }
  }

  // Save gScore_
  if (sharedConfig_->saveGScore) {
    outputFilePath = "./output/" + methodName + "_gScore.txt";
    std::fstream of1(outputFilePath, std::ios::out | std::ios::trunc);
    if (!of1.is_open()) {
      std::cerr << "Failed to open output file " << outputFilePath << std::endl;
      return;
    }
    std::ostream &os = of1;
    for (int j = ny_ - 1; j >= 0; --j) {
      for (size_t i = 0; i < nx_; ++i) {
        os << gScore_(i, j) << " ";
      }
      os << "\n";
    }
    of1.close();
    if (!sharedConfig_->silent) {
      std::cout << "Saved " + methodName + " gScore" << std::endl;
    }
  }

  // Save fScore_
  if (sharedConfig_->saveFScore) {
    outputFilePath = "./output/" + methodName + "_fScore.txt";
    std::fstream of2(outputFilePath, std::ios::out | std::ios::trunc);
    if (!of2.is_open()) {
      std::cerr << "Failed to open output file " << outputFilePath << std::endl;
      return;
    }
    std::ostream &os = of2;
    for (int j = ny_ - 1; j >= 0; --j) {
      for (size_t i = 0; i < nx_; ++i) {
        os << fScore_(i, j) << " ";
      }
      os << "\n";
    }
    of2.close();
    if (!sharedConfig_->silent) {
      std::cout << "Saved " + methodName + " fScore" << std::endl;
    }
  }

  // Save lightsources
  if (sharedConfig_->saveLightSources) {
    outputFilePath = "./output/" + methodName + "_lightSources.txt";
    std::fstream of3(outputFilePath, std::ios::out | std::ios::trunc);
    if (!of3.is_open()) {
      std::cerr << "Failed to open output file " << outputFilePath << std::endl;
      return;
    }
    std::ostream &os = of3;
    for (size_t i = 0; i < nb_of_sources_; ++i) {
      os << lightSources_[i].x << " " << ny_ - 1 - lightSources_[i].y;
      os << "\n";
    }
    of3.close();
    if (!sharedConfig_->silent) {
      std::cout << "Saved " + methodName + " lightSources" << std::endl;
    }
  }
}

} // namespace vbm
