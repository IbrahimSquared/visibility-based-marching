#include "solver/solver.h"

#include <fstream>
#include <sstream>
#include <chrono>

namespace vbs {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
solver::solver(environment& env) : sharedConfig_(env.getConfig()) {
  sharedVisibilityField_ = env.getVisibilityField();
  sharedSpeedField_ = env.getSpeedField();
  ncols_ = sharedVisibilityField_.ncols();
  nrows_ = sharedVisibilityField_.nrows();
  visibilityThreshold_ = sharedConfig_->visibilityThreshold;

  // Init environment image
  uniqueLoadedImage_.reset(std::make_unique<sf::Image>().release());
  uniqueLoadedImage_->create(ncols_, nrows_, sf::Color::Black);
  sf::Color color;
  color.a = 1;
  for (size_t i = 0; i < ncols_; ++i) {
    for (size_t j = 0; j < nrows_; ++j) {
      if (sharedVisibilityField_(i,j) < 1) {
        uniqueLoadedImage_->setPixel(i, j, color.Black);
      } else {
        uniqueLoadedImage_->setPixel(i, j, color.White);
      }
    }
  }
  // Init maps
  reset();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void solver::reset() {
  gScore_ = Field<double, 0>(ncols_, nrows_, INFINITY);
  fScore_ = Field<double, 0>(ncols_, nrows_, INFINITY);
  cameFrom_ = Field<size_t, 0>(ncols_, nrows_, 0);
  inOpenSet_ = Field<bool, 0>(ncols_, nrows_, false);
  isUpdated_ = Field<bool, 0>(ncols_, nrows_, false);
  lightSources_.reset(new point[ncols_ * nrows_]);

  visibilityHashMap_.clear();
  openSet_.reset();

  // Reserve openSet_
  std::vector<Node> container;
  container.reserve(ncols_*nrows_);
  std::priority_queue<Node, std::vector<Node>, std::less<Node>> heap(std::less<Node>(), std::move(container));
  openSet_ = std::make_unique<std::priority_queue<Node>>(heap); 

  nb_of_sources_ = 0;
  nb_of_iterations_ = 0;
  
  // Reserve hash map
  visibilityHashMap_.reserve(ncols_ * nrows_ * 2);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void solver::visibilityBasedSolver() {
  reset();
  auto startTime = std::chrono::high_resolution_clock::now();

  // Init
  double d = 0;
  int x = 0, y = 0, neighbour_x = 0, neighbour_y = 0;

  auto& initial_frontline = sharedConfig_->initialFrontline;
  // Fill in data from initial frontline
  for(size_t i = 0; i < initial_frontline.size(); i += 2) {
    x = initial_frontline[i]; y = nrows_ - 1 - initial_frontline[i+1];
    d = 0;
    gScore_(x, y) = d;
    isUpdated_(x, y) = true;
    cameFrom_(x, y) = nb_of_sources_;
    lightSources_[nb_of_sources_] = {x, y};

    openSet_->push(Node{x, y, d});
    visibilityHashMap_[y + ncols_ * x + nrows_ * ncols_ * nb_of_sources_] = lightStrength_;
    ++nb_of_sources_;
    ++nb_of_iterations_;
  }

  // For queing unique sources from neighbours of neighbour
  std::vector<size_t> potentialSources; potentialSources.reserve(10);
  std::vector<std::pair<double, size_t>> potentialDistances; potentialDistances.reserve(10);

  double distance = 0;

  while (openSet_->size() > 0) {
    auto& current = openSet_->top();
    x = current.x;
    y = current.y;
    openSet_->pop();

    // Expand frontline at current & update neighbours
    for (size_t j = 0; j < 16; j += 2) {
      // NOTE those are always positive
      neighbour_x = x + neighbours_[j];
      neighbour_y = y + neighbours_[j+1];

      // Box check
      if (neighbour_x >= ncols_ || neighbour_y >= nrows_) { continue; };
      if (isUpdated_(neighbour_x, neighbour_y)) { continue; };
      if (sharedVisibilityField_(neighbour_x, neighbour_y) < 1) {
        if (sharedConfig_->expandInObstacles) {
          gScore_(neighbour_x, neighbour_y) = gScore_(x, y) + evaluateDistanceSpeedField(x, y, neighbour_x, neighbour_y);
          openSet_->push(Node{neighbour_x, neighbour_y, gScore_(neighbour_x, neighbour_y)});
          visibilityHashMap_[y + ncols_ * x + nrows_ * ncols_ * nb_of_sources_] = sharedVisibilityField_(neighbour_x, neighbour_y);;
        }
        cameFrom_(neighbour_x, neighbour_y) = cameFrom_(x, y);
        isUpdated_(neighbour_x, neighbour_y) = true; 
        continue; 
      }
      
      potentialSources = queuePotentialSources(potentialSources, neighbour_x, neighbour_y);
      potentialDistances = getPotentialDistancesSpeedField(potentialSources, potentialDistances, neighbour_x, neighbour_y);

      auto minimum_element = std::min_element(potentialDistances.begin(), potentialDistances.end(),
        [](const auto& lhs, const auto& rhs) { 
        return lhs.first < rhs.first;
      });
      distance = minimum_element->first;

      if (distance == INFINITY) {
        createNewPivot(x, y, neighbour_x, neighbour_y);
      } else {
        // use source giving least distance
        gScore_(neighbour_x, neighbour_y) = minimum_element->first;
        cameFrom_(neighbour_x, neighbour_y) = minimum_element->second;
      }
      openSet_->push(Node{neighbour_x, neighbour_y, gScore_(neighbour_x, neighbour_y)});
      isUpdated_(neighbour_x, neighbour_y) = true;
      ++nb_of_iterations_;
    }
  };

  auto stopTime = std::chrono::high_resolution_clock::now();
  auto executionDuration = std::chrono::duration_cast<std::chrono::microseconds>(stopTime - startTime);
  if (!sharedConfig_->silent) {
    std::cout << "############################## Visibility-based solver output ##############################" << std::endl;
    if (sharedConfig_->timer) {
      std::cout << "Execution time in us: " << executionDuration.count() << "us" << std::endl;
      std::cout << "Load factor: " << visibilityHashMap_.load_factor() << std::endl;
      std::cout << "Iterations: " << nb_of_iterations_ << std::endl;
      std::cout << "Nb of sources: " << nb_of_sources_ << std::endl;
    }
  }
  if (sharedConfig_->saveResults) {
    saveResults({}, "visibilityBased");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void solver::vStarSearch() {
  reset();
  auto startTime = std::chrono::high_resolution_clock::now();

  // Init
  double g = 0, h = 0, f = 0;
  int x = 0, y = 0, neighbour_x = 0, neighbour_y = 0;
  int endX = sharedConfig_->target_x;
  int endY = nrows_ - 1 - sharedConfig_->target_y;

  auto& initial_frontline = sharedConfig_->initialFrontline;
  // Fill in data from initial frontline
  for(size_t i = 0; i < initial_frontline.size(); i += 2) {
    x = initial_frontline[i]; y = nrows_ - 1 - initial_frontline[i+1];
    g = 0; h = 0;
    if (sharedConfig_->greedy) { h = evaluateDistance(x, y, endX, endY); }
    f = g + h;
    openSet_->push(Node{x, y, f});

    gScore_(x, y) = g;
    fScore_(x, y) = f;
    cameFrom_(x, y) = nb_of_sources_;
    isUpdated_(x, y) = true;
    lightSources_[nb_of_sources_] = {x, y};
    visibilityHashMap_[y + ncols_ * x + nrows_ * ncols_ * nb_of_sources_] = lightStrength_;
    ++nb_of_sources_;
    ++nb_of_iterations_;
  }

  // For queing unique sources from neighbours of neighbour
  std::vector<size_t> potentialSources; potentialSources.reserve(10);
  std::vector<std::pair<double, size_t>> potentialDistances; potentialDistances.reserve(10);

  double distance = 0;

  while (openSet_->size() > 0) {
    auto& current = openSet_->top();
    x = current.x;
    y = current.y;
    if (x == endX && y == endY) {
      auto stopTime = std::chrono::high_resolution_clock::now();
      auto executionDuration = std::chrono::duration_cast<std::chrono::microseconds>(stopTime - startTime);
      
      if (sharedConfig_->silent) { return; }
      std::cout << "############################## VStar solver output ##############################" << std::endl;
      std::cout << "Path found" << std::endl;
      if (!sharedConfig_->timer) { return; }
      std::cout << "Execution time in us: " << executionDuration.count() << "us" << std::endl;
      std::cout << "Load factor: " << visibilityHashMap_.load_factor() << std::endl;
      std::cout << "Iterations: " << nb_of_iterations_ << std::endl;
      reconstructPath(current, "vstar");
      return;
    }

    openSet_->pop();

    // Expand frontline at current & update neighbours
    for (size_t j = 0; j < 16; j += 2) {
      // NOTE those are always positive
      neighbour_x = x + neighbours_[j];
      neighbour_y = y + neighbours_[j+1];

      // Box check
      if (neighbour_x >= ncols_ || neighbour_y >= nrows_) { continue; };
      if (isUpdated_(neighbour_x, neighbour_y)) { continue; };
      if (sharedVisibilityField_(neighbour_x, neighbour_y) < 1) {
        cameFrom_(neighbour_x, neighbour_y) = cameFrom_(x, y);
        isUpdated_(neighbour_x, neighbour_y) = true; 
        continue; 
      }
      
      potentialSources = queuePotentialSources(potentialSources, neighbour_x, neighbour_y);
      potentialDistances = getPotentialDistances(potentialSources, potentialDistances, neighbour_x, neighbour_y);

      auto minimum_element = std::min_element(potentialDistances.begin(), potentialDistances.end(),
        [](const auto& lhs, const auto& rhs) { 
        return lhs.first < rhs.first;
      });
      distance = minimum_element->first;

      if (distance == INFINITY) {
        createNewPivot(x, y, neighbour_x, neighbour_y);
      } else {
        // use source giving least distance
        gScore_(neighbour_x, neighbour_y) = minimum_element->first;
        cameFrom_(neighbour_x, neighbour_y) = minimum_element->second;
      }

      h = 0;
      if (sharedConfig_->greedy) { h = evaluateDistance(neighbour_x, neighbour_y, endX, endY); }
      f = gScore_(neighbour_x, neighbour_y) + h;
      fScore_(neighbour_x, neighbour_y) = f;

      openSet_->push(Node{neighbour_x, neighbour_y, f});
      isUpdated_(neighbour_x, neighbour_y) = true;
      ++nb_of_iterations_;
    }
  }

  auto stopTime = std::chrono::high_resolution_clock::now();
  auto executionDuration = std::chrono::duration_cast<std::chrono::microseconds>(stopTime - startTime);
  if (!sharedConfig_->silent) {
    std::cout << "############################## VStar solver output ##############################" << std::endl;
    if (sharedConfig_->timer) {
      std::cout << "Path could not be found" << std::endl;
      std::cout << "Execution time in us: " << executionDuration.count() << "us" << std::endl;
      std::cout << "Load factor: " << visibilityHashMap_.load_factor() << std::endl;
      std::cout << "Iterations: " << nb_of_iterations_ << std::endl;
      reconstructPath({}, "vstar");
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void solver::aStarSearch() {
  reset();
  auto startTime = std::chrono::high_resolution_clock::now();

  // Init
  double g = 0, h = 0, f = 0;
  int x = 0, y = 0, neighbour_x = 0, neighbour_y = 0;
  int endX = sharedConfig_->target_x;
  int endY = nrows_ - 1 - sharedConfig_->target_y;

  auto& initial_frontline = sharedConfig_->initialFrontline;
  // Fill in data from initial frontline
  for(size_t i = 0; i < initial_frontline.size(); i += 2) {
    x = initial_frontline[i]; y = nrows_ - 1 - initial_frontline[i+1];
    g = 0; h = 0;
    if (sharedConfig_->greedy) { h = evaluateDistance(x, y, endX, endY); }
    f = g + h;
    openSet_->push(Node{x, y, f});
    inOpenSet_(x, y) = true;
    
    gScore_(x, y) = g;
    fScore_(x, y) = f;
    cameFrom_(x, y) = indexAt(x, y);
    ++nb_of_iterations_;
  }

  while (openSet_->size() > 0) {
    auto& current = openSet_->top();
    x = current.x;
    y = current.y;

    if (x == endX && y == endY) {
      auto stopTime = std::chrono::high_resolution_clock::now();
      auto executionDuration = std::chrono::duration_cast<std::chrono::microseconds>(stopTime - startTime);
      
      if (sharedConfig_->silent) { return; }
      std::cout << "############################## AStar solver output ##############################" << std::endl;
      std::cout << "Path found" << std::endl;
      if (!sharedConfig_->timer) { return; }
      std::cout << "Execution time in us: " << executionDuration.count() << "us" << std::endl;
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
      neighbour_y = y + neighbours_[j+1];

      // Box check
      if (neighbour_x >= ncols_ || neighbour_y >= nrows_ || neighbour_x < 0 || neighbour_y < 0) { continue; };
      if (sharedVisibilityField_(neighbour_x, neighbour_y) < 1) { continue; }

      g = gScore_(x, y) + evaluateDistance(x, y, neighbour_x, neighbour_y); // neighbour_distances_[j];
      if (g >= gScore_(neighbour_x, neighbour_y)) { continue; };

      cameFrom_(neighbour_x, neighbour_y) = indexAt(x, y);
      gScore_(neighbour_x, neighbour_y) = g;
      if (sharedConfig_->greedy) { h = evaluateDistance(neighbour_x, neighbour_y, endX, endY); }
      f = gScore_(neighbour_x, neighbour_y) + h;
      fScore_(neighbour_x, neighbour_y) = f;
      ++nb_of_iterations_;
      if (inOpenSet_(neighbour_x, neighbour_y)) { continue; }

      openSet_->push(Node{neighbour_x, neighbour_y, f});
      inOpenSet_(neighbour_x, neighbour_y) = true;
    }
  };

  auto stopTime = std::chrono::high_resolution_clock::now();
  auto executionDuration = std::chrono::duration_cast<std::chrono::microseconds>(stopTime - startTime);
  if (!sharedConfig_->silent) {
    std::cout << "############################## AStar Solver output ##############################" << std::endl;
    if (sharedConfig_->timer) {
      std::cout << "Path could not be found" << std::endl;
      std::cout << "Execution time in us: " << executionDuration.count() << "us" << std::endl;
      std::cout << "Iterations: " << nb_of_iterations_ << std::endl;
      reconstructPath({}, "astar");
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void solver::computeDistanceFunction() {
  reset();
  auto startTime = std::chrono::high_resolution_clock::now();
  std::vector<int> initial_frontline;
  for (size_t i = 0; i < ncols_; ++i) {
    for (size_t j = 0; j < nrows_; ++j) {
      if (sharedVisibilityField_(i, j) <= visibilityThreshold_) {
        initial_frontline.push_back(i);
        initial_frontline.push_back(j);
      }
    }
  }
  // Init
  int x = 0, y = 0, neighbour_x = 0, neighbour_y = 0;
  double g = 0, f = 0;
  // Fill in data from initial frontline
  for(size_t i = 0; i < initial_frontline.size(); i += 2) {
    x = initial_frontline[i]; y = initial_frontline[i+1];
    g = 0;
    f = g;
    openSet_->push(Node{x, y, f});
    inOpenSet_(x, y) = true;

    gScore_(x, y) = g;
    fScore_(x, y) = f;
    isUpdated_(x, y) = true;
    cameFrom_(x, y) = nb_of_sources_;
    lightSources_[nb_of_sources_] = {x, y};
    visibilityHashMap_[y + ncols_ * x + nrows_ * ncols_ * nb_of_sources_] = lightStrength_;
    ++nb_of_sources_;
  }

  // For queing unique sources from neighbours of neighbour
  std::vector<size_t> potentialSources; potentialSources.reserve(10);
  std::vector<std::pair<double, size_t>> potentialDistances; potentialDistances.reserve(10);

  double distance = 0;

  while (openSet_->size() > 0) {
    auto& current = openSet_->top();
    x = current.x; y = current.y;

    openSet_->pop();
    inOpenSet_(x, y) = false;

    // Expand frontline at current & update neighbours
    for (size_t j = 0; j < 16; j += 2) {
      // NOTE those are always positive
      neighbour_x = x + neighbours_[j];
      neighbour_y = y + neighbours_[j+1];

      // Box check
      if (neighbour_x >= ncols_ || neighbour_y >= nrows_) { continue; };
      if (isUpdated_(neighbour_x, neighbour_y)) { continue; };
      
      potentialSources = queuePotentialSources(potentialSources, neighbour_x, neighbour_y);
      potentialDistances = getPotentialDistances(potentialSources, potentialDistances, neighbour_x, neighbour_y);

      auto minimum_element = std::min_element(potentialDistances.begin(), potentialDistances.end(),
        [](const auto& lhs, const auto& rhs) { 
        return lhs.first < rhs.first;
      });
      distance = minimum_element->first;

      if (distance == INFINITY) {
        createNewPivot(x, y, neighbour_x, neighbour_y);
      } else {
        // use source giving least distance
        gScore_(neighbour_x, neighbour_y) = minimum_element->first;
        cameFrom_(neighbour_x, neighbour_y) = minimum_element->second;
      }

      f = gScore_(neighbour_x, neighbour_y);
      fScore_(neighbour_x, neighbour_y) = f;

      if (!inOpenSet_(neighbour_x, neighbour_y)) {
        openSet_->push(Node{neighbour_x, neighbour_y, f});
        inOpenSet_(neighbour_x, neighbour_y) = true;
      }

      isUpdated_(neighbour_x, neighbour_y) = true;
    }
  };

  auto stopTime = std::chrono::high_resolution_clock::now();
  auto executionDuration = std::chrono::duration_cast<std::chrono::microseconds>(stopTime - startTime);
  if (!sharedConfig_->silent) {
    std::cout << "############################## Distance function computation output ##############################" << std::endl;
    if (sharedConfig_->timer) {
      std::cout << "Constructed distance function" << std::endl;
      std::cout << "Execution time in us: " << executionDuration.count() << "us" << std::endl;
    }
  }
  if (sharedConfig_->saveResults) {
    saveResults({}, "distanceFunction");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<size_t>& solver::queuePotentialSources(std::vector<size_t>& potentialSources, const int neighbour_x, const int neighbour_y) const {
  size_t potentialSource_x = 0, potentialSource_y = 0, lightSource_num = 0;
  potentialSources.clear();
  // Queue sources from updated neighbours of neighbour
  for (size_t k = 0; k < 16; k += 2) {
    // NOTE those are always positive
    potentialSource_x = neighbour_x + neighbours_[k];
    potentialSource_y = neighbour_y + neighbours_[k+1];
    // Box check
    if (potentialSource_x >= ncols_ || potentialSource_y >= nrows_) { continue; };
    if (!isUpdated_(potentialSource_x, potentialSource_y)) { continue; };

    lightSource_num = cameFrom_(potentialSource_x, potentialSource_y);
    // Pick only unique sources (no repitition in potentialSources)
    if (std::find(potentialSources.begin(), potentialSources.end(), lightSource_num) == potentialSources.end()) {
      potentialSources.push_back(lightSource_num);
    }
  }
  return potentialSources;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<std::pair<double, size_t>>& solver::getPotentialDistances(const std::vector<size_t>& potentialSources, std::vector<std::pair<double, size_t>>& potentialDistances, const int neighbour_x, const int neighbour_y) {
  size_t lightSource_x = 0, lightSource_y = 0, potentialSource = 0;
  double distance = 0;
  potentialDistances.clear();
  for (size_t k = 0; k < potentialSources.size(); ++k) {
    potentialSource = potentialSources[k];
    lightSource_x = lightSources_[potentialSource].first;
    lightSource_y = lightSources_[potentialSource].second;
    // update visibility from source
    updatePointVisibility(potentialSource, lightSource_x, lightSource_y, neighbour_x, neighbour_y);
    distance = INFINITY;
    if (visibilityHashMap_.at(neighbour_y + ncols_ * neighbour_x + nrows_ * ncols_ * potentialSource) >= visibilityThreshold_) {
      distance = gScore_(lightSource_x, lightSource_y) + evaluateDistance(lightSource_x, lightSource_y, neighbour_x, neighbour_y);
    }
    potentialDistances.push_back(std::pair<double, int>{distance, potentialSource});
  }
  return potentialDistances;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<std::pair<double, size_t>>& solver::getPotentialDistancesSpeedField(const std::vector<size_t>& potentialSources, std::vector<std::pair<double, size_t>>& potentialDistances, const int neighbour_x, const int neighbour_y) {
  size_t lightSource_x = 0, lightSource_y = 0, potentialSource = 0;
  double distance = 0;
  potentialDistances.clear();
  for (size_t k = 0; k < potentialSources.size(); ++k) {
    potentialSource = potentialSources[k];
    lightSource_x = lightSources_[potentialSource].first;
    lightSource_y = lightSources_[potentialSource].second;
    // update visibility from source
    updatePointVisibility(potentialSource, lightSource_x, lightSource_y, neighbour_x, neighbour_y);
    distance = INFINITY;
    if (visibilityHashMap_.at(neighbour_y + ncols_ * neighbour_x + nrows_ * ncols_ * potentialSource) >= visibilityThreshold_) {
      distance = gScore_(lightSource_x, lightSource_y) + evaluateDistanceSpeedField(lightSource_x, lightSource_y, neighbour_x, neighbour_y);
    }
    potentialDistances.push_back(std::pair<double, int>{distance, potentialSource});
  }
  return potentialDistances;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void solver::createNewPivot(const int x, const int y, const int neighbour_x, const int neighbour_y) {
  int pivot_neighbour_x, pivot_neighbour_y;
  // Pushback parent as a new lightSource
  lightSources_[nb_of_sources_] = std::make_pair(x, y); // {x, y};
  // Pusback pivot & update light source visibility
  visibilityHashMap_[y + ncols_ * x + nrows_ * ncols_ * nb_of_sources_] = lightStrength_;
  // Update maps of new pivot_
  // Update neighbours of initial frontline points - both distance & visibility
  for(size_t p = 0; p < 16; p += 2) {
    // NOTE those are always positive
    pivot_neighbour_x = x + neighbours_[p];
    pivot_neighbour_y = y + neighbours_[p+1];
    // Box check
    if (pivot_neighbour_x >= ncols_ || pivot_neighbour_y >= nrows_) { continue; }
    // Update neighbour visibility
    updatePointVisibility(nb_of_sources_, x, y, pivot_neighbour_x, pivot_neighbour_y);
    visibilityHashMap_[pivot_neighbour_y + ncols_ * pivot_neighbour_x + nrows_ * ncols_ * nb_of_sources_] = visibilityHashMap_.at(pivot_neighbour_y + ncols_ * pivot_neighbour_x + nrows_ * ncols_ * nb_of_sources_);
  }
  cameFrom_(neighbour_x, neighbour_y) = nb_of_sources_;
  gScore_(neighbour_x, neighbour_y) = gScore_(x, y) + evaluateDistance(x, y, neighbour_x, neighbour_y);
  ++nb_of_sources_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void solver::updatePointVisibility(const size_t lightSourceNumber, const int lightSource_x, const int lightSource_y, const int x, const int y) {
  // Variable initialization
  double v = 0;
  double c = 0;

  // Check if visibility value already exists
  size_t key = y + ncols_ * x + nrows_ * ncols_ * lightSourceNumber;
  if (visibilityHashMap_.count(key)) {
    return;
  }

  if (x == lightSource_x) {
    if (y - lightSource_y > 0) { 
      key = (y-1) + ncols_ * x + nrows_ * ncols_ * lightSourceNumber;
      if (!visibilityHashMap_.count(key)) {
        updatePointVisibility(lightSourceNumber, lightSource_x, lightSource_y, x, y-1);
      } 
      v = visibilityHashMap_.at(key);
    } else {
      key = y+1 + ncols_ * x + nrows_ * ncols_ * lightSourceNumber;;
      if (!visibilityHashMap_.count(key)) {
        updatePointVisibility(lightSourceNumber, lightSource_x, lightSource_y, x, y+1);
      } 
      v = visibilityHashMap_.at(key);
    }
  } else if (y == lightSource_y) {
    if (x - lightSource_x > 0) { 
      key = y + ncols_ * (x-1) + nrows_ * ncols_ * lightSourceNumber;
      if (!visibilityHashMap_.count(key)) {
        updatePointVisibility(lightSourceNumber, lightSource_x, lightSource_y, x-1, y);
      } 
      v = visibilityHashMap_.at(key);
    } else {
      key = y + ncols_ * (x+1) + nrows_ * ncols_ * lightSourceNumber;
      if (!visibilityHashMap_.count(key)) {
        updatePointVisibility(lightSourceNumber, lightSource_x, lightSource_y, x+1, y);
      } 
      v = visibilityHashMap_.at(key);
    }
  } else {
    // Q1
    if ((x - lightSource_x > 0) && (y - lightSource_y > 0)) {
      key = (y-1) + ncols_ * (x-1) + nrows_ * ncols_ * lightSourceNumber;
      if (!visibilityHashMap_.count(key)) {
        updatePointVisibility(lightSourceNumber, lightSource_x, lightSource_y, x-1, y-1);
      }
      if (x - lightSource_x == y - lightSource_y) {
        v = visibilityHashMap_.at(key);
      } else if (x - lightSource_x < y - lightSource_y) {
        const size_t key_1 = (y-1) + ncols_ * x + nrows_ * ncols_ * lightSourceNumber;
        if (!visibilityHashMap_.count(key_1)) {
          updatePointVisibility(lightSourceNumber, lightSource_x, lightSource_y, x, y-1);
        }
        c = static_cast<double>(x - lightSource_x) / (y - lightSource_y);
        double v1 = visibilityHashMap_.at(key_1);
        v = v1 - c * (v1 - visibilityHashMap_.at(key));
      } else if (x - lightSource_x > y - lightSource_y) {
        const size_t key_2 = y + ncols_ * (x-1) + nrows_ * ncols_ * lightSourceNumber;
        if (!visibilityHashMap_.count(key_2)) {
          updatePointVisibility(lightSourceNumber, lightSource_x, lightSource_y, x-1, y);
        }
        c = static_cast<double>(y - lightSource_y) / (x - lightSource_x);
        double v2 = visibilityHashMap_.at(key_2);
        v = v2 - c * (v2 - visibilityHashMap_.at(key));
      }
    } 
    // Q2
    else if ((x - lightSource_x > 0) && (y - lightSource_y < 0)) {
      key = (y+1) + ncols_ * (x-1) + nrows_ * ncols_ * lightSourceNumber;
      if (!visibilityHashMap_.count(key)) {
        updatePointVisibility(lightSourceNumber, lightSource_x, lightSource_y, x-1, y+1);
      }
      if (x - lightSource_x == lightSource_y - y) {
        v = visibilityHashMap_.at(key);
      } else if (x - lightSource_x < lightSource_y - y) {
        const size_t key_1 = y+1 + ncols_ * x + nrows_ * ncols_ * lightSourceNumber;
        if (!visibilityHashMap_.count(key_1)) {
          updatePointVisibility(lightSourceNumber, lightSource_x, lightSource_y, x, y+1);
        }
        c = static_cast<double>(x - lightSource_x) / (lightSource_y - y);
        double v1 = visibilityHashMap_.at(key_1);
        v = v1 - c * (v1 - visibilityHashMap_.at(key));
      } else if (x - lightSource_x > lightSource_y - y) {
        const size_t key_2 = y + ncols_ * (x-1) + nrows_ * ncols_ * lightSourceNumber;
        if (!visibilityHashMap_.count(key_2)) {
          updatePointVisibility(lightSourceNumber, lightSource_x, lightSource_y, x-1, y);
        }
        c = static_cast<double>(lightSource_y - y) / (x - lightSource_x);
        double v2 = visibilityHashMap_.at(key_2);
        v = v2 - c * (v2 - visibilityHashMap_.at(key));
      }
    } 
    // Q3
    else if ((x - lightSource_x < 0) && (y - lightSource_y < 0)) {
      key = (y+1) + ncols_ * (x+1) + nrows_ * ncols_ * lightSourceNumber;
      if (!visibilityHashMap_.count(key)) {
        updatePointVisibility(lightSourceNumber, lightSource_x, lightSource_y, x+1, y+1);
      }
      if (lightSource_x - x == lightSource_y - y) {
        v = visibilityHashMap_.at(key);
      } else if (lightSource_x - x < lightSource_y - y) {
        const size_t key_1 = (y+1) + ncols_ * x + nrows_ * ncols_ * lightSourceNumber;
        if (!visibilityHashMap_.count(key_1)) {
          updatePointVisibility(lightSourceNumber, lightSource_x, lightSource_y, x, y+1);
        }
        c = static_cast<double>(lightSource_x - x) / (lightSource_y - y);
        double v1 = visibilityHashMap_.at(key_1);
        v = v1 - c * (v1 - visibilityHashMap_.at(key));
      } else if (lightSource_x - x > lightSource_y - y) {
        const size_t key_2 = y + ncols_ * (x+1) + nrows_ * ncols_ * lightSourceNumber;
        if (!visibilityHashMap_.count(key_2)) {
          updatePointVisibility(lightSourceNumber, lightSource_x, lightSource_y, x+1, y);
        }
        c = static_cast<double>(lightSource_y - y) / (lightSource_x - x);
        double v2 = visibilityHashMap_.at(key_2);
        v = v2 - c * (v2 - visibilityHashMap_.at(key));
      }
    } 
    // Q4
    else if ((x - lightSource_x < 0) && (y - lightSource_y > 0)) {
      key = (y-1) + ncols_ * (x+1) + nrows_ * ncols_ * lightSourceNumber;
      if (!visibilityHashMap_.count(key)) {
        updatePointVisibility(lightSourceNumber, lightSource_x, lightSource_y, x+1, y-1);
      }
      if (lightSource_x - x == y - lightSource_y) {
        v = visibilityHashMap_.at(key);
      } else if (lightSource_x - x < y - lightSource_y) {
        const size_t key_1 = (y-1) + ncols_ * x + nrows_ * ncols_ * lightSourceNumber;
        if (!visibilityHashMap_.count(key_1)) {
          updatePointVisibility(lightSourceNumber, lightSource_x, lightSource_y, x, y-1);
        }
        c = static_cast<double>(lightSource_x - x) / (y - lightSource_y);
        double v1 = visibilityHashMap_.at(key_1);
        v = v1 - c * (v1 - visibilityHashMap_.at(key));
      } else if (lightSource_x - x > y - lightSource_y) {
        const size_t key_2 = y + ncols_ * (x+1) + nrows_ * ncols_ * lightSourceNumber;
        if (!visibilityHashMap_.count(key_2)) {
          updatePointVisibility(lightSourceNumber, lightSource_x, lightSource_y, x+1, y);
        }
        c = static_cast<double>(y - lightSource_y) / (lightSource_x - x);
        double v2 = visibilityHashMap_.at(key_2);
        v = v2 - c * (v2 - visibilityHashMap_.at(key));
      }
    }
  }
  v = v * sharedVisibilityField_(x, y);
  visibilityHashMap_[y + ncols_ * x + nrows_ * ncols_ * lightSourceNumber] = v;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void solver::reconstructPath(const Node& current, const std::string& methodName) {
  std::vector<point> resultingPath;
  int x = current.x, y = current.y;
  double t = cameFrom_(x, y);
  double t_old = INFINITY;
  while (t != t_old) {
    resultingPath.push_back({x, y});
    t_old = t;
    if (methodName == "vstar") {
      x = lightSources_[t].first; y = lightSources_[t].second;
    } else if (methodName == "astar") {
      auto p = coordinatesAt(t);
      x = p.first; y = p.second;
    }
    t = cameFrom_(x, y);
  }
  resultingPath.push_back({x, y});
  std::reverse(resultingPath.begin(), resultingPath.end());

  if (sharedConfig_->saveResults) {
    saveImageWithPath(resultingPath, methodName);
    saveResults(resultingPath, methodName);
  }
  return;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void solver::saveImageWithPath(const std::vector<point>& path, const std::string& methodName) {
  sf::Image image;
  image = *uniqueLoadedImage_;
  sf::Color color;
  color.a = 1;
  int x0, y0, x1, y1;
  int dx, dy, sx, sy, err;
  
  for (size_t i = 0; i < path.size() - 1; ++i) {
    x0 = path[i].first; y0 = path[i].second;
    x1 = path[i+1].first; y1 = path[i+1].second;

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
  std::string imageName = "output/" + methodName + ".png";
  image.saveToFile(imageName);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void solver::saveResults(const std::vector<point>& resultingPath, const std::string& methodName) {
  namespace fs = std::filesystem;

  // Define the path to the output file
  std::string outputFilePath = "./output/" + methodName + ".txt";

  // Check if the directory exists, and create it if it doesn't
  fs::path directory = fs::path(outputFilePath).parent_path();
  if (!fs::exists(directory)) {
    if (!fs::create_directories(directory)) {
      std::cerr << "Failed to create directory " << directory.string() << std::endl;
      return;
    }
  }

  if (methodName == "distanceFunction") {
    // save distance function
    if (sharedConfig_->saveDistanceFunction) {
      std::fstream  of(outputFilePath, std::ios::out | std::ios::trunc);
      if (!of.is_open()) {
        std::cerr << "Failed to open output file " << outputFilePath << std::endl;
        return;
      }
      std::ostream& os = of;
      for (int j = nrows_ - 1; j >= 0; --j){
        for (size_t i = 0; i < ncols_; ++i) {
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
    std::fstream  of(outputFilePath, std::ios::out | std::ios::trunc);
    if (!of.is_open()) {
      std::cerr << "Failed to open output file " << outputFilePath << std::endl;
      return;
    }
    std::ostream& os = of;
    for (int j = nrows_ - 1; j >= 0; --j){
      for (size_t i = 0; i < ncols_; ++i) {
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
      std::fstream  of1(outputFilePath, std::ios::out | std::ios::trunc);
      if (!of1.is_open()) {
        std::cerr << "Failed to open output file " << outputFilePath << std::endl;
        return;
      }
      std::ostream& os1 = of1;
      for (int j = nrows_ - 1; j >= 0; --j){
        for (size_t i = 0; i < ncols_; ++i) {
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
        std::cerr << "Failed to open output file " << outputFilePath << std::endl;
        return;
      }
      std::ostream& os = of3;
      for (size_t i = 0; i < nb_of_sources_; ++i) { 
        os << lightSources_[i].first << " " << nrows_- 1 - lightSources_[i].second ; 
        os<< "\n";
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
    std::ostream& os = of;
    for (size_t i = 0; i < resultingPath.size(); ++i) { 
      os << resultingPath[i].first << " " << nrows_ - 1 - resultingPath[i].second ; 
      os<< "\n";
    }
    of.close();
    if (!sharedConfig_->silent) {
      std::cout << "Saved " + methodName + " path"  << std::endl;
    }
  }

  // Save gScore_
  if (sharedConfig_->saveGScore) {
    outputFilePath = "./output/" + methodName + "_gScore.txt";
    std::fstream  of1(outputFilePath, std::ios::out | std::ios::trunc);
    if (!of1.is_open()) {
      std::cerr << "Failed to open output file " << outputFilePath << std::endl;
      return;
    }
    std::ostream& os = of1;
    for (int j = nrows_ - 1; j >= 0; --j){
      for (size_t i = 0; i < ncols_; ++i) {
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
    std::fstream  of2(outputFilePath, std::ios::out | std::ios::trunc);
    if (!of2.is_open()) {
      std::cerr << "Failed to open output file " << outputFilePath << std::endl;
      return;
    }
    std::ostream& os = of2;
    for (int j = nrows_ - 1; j >= 0; --j){
      for (size_t i = 0; i < ncols_; ++i) {
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
    std::ostream& os = of3;
    for (size_t i = 0; i < nb_of_sources_; ++i) { 
      os << lightSources_[i].first << " " << nrows_ - 1 - lightSources_[i].second ; 
      os<< "\n";
    }
    of3.close();
    if (!sharedConfig_->silent) {
      std::cout << "Saved " + methodName + " lightSources" << std::endl;
    }
  }
}

} // namespace vbs