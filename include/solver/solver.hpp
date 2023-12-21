#ifndef SOLVER_HPP
#define SOLVER_HPP

#include <cmath>
#include <queue>
#include <vector>

#include "environment/environment.hpp"
#include "flat_hash_map/flat_hash_map.hpp"

namespace vbs {

struct Node {
  int x, y;
  double f;
  bool operator<(const Node &other) const { return f > other.f; }
};

class Solver {
public:
  using Map = ska::flat_hash_map<size_t, double>;
  using point = std::pair<size_t, size_t>;

  explicit Solver(Environment &env);

  // Deconstructor
  ~Solver() = default;

  void vStarSearch();
  void aStarSearch();
  void visibilityBasedSolver();
  void computeDistanceFunction();

  // Get global iterator (number of iterations that had to be completed)
  inline int getNbOfIterations() const { return nb_of_iterations_; };

private:
  void reset();
  void reconstructPath(const Node &current, const std::string &methodName);
  inline int indexAt(const size_t x, const size_t y) const {
    return x + y * nx_;
  };
  inline point coordinatesAt(size_t index) const {
    size_t x = index % nx_;
    size_t y = (index) / nx_;
    return {x, y};
  }

  /*!
   * @brief queues sources
   */
  inline std::vector<size_t> &
  queuePotentialSources(std::vector<size_t> &potentialSources,
                        const int neighbour_x, const int neighbour_y) const;

  /*!
   * @brief gets distances
   */
  inline std::vector<std::pair<double, size_t>> &getPotentialDistances(
      const std::vector<size_t> &potentialSources,
      std::vector<std::pair<double, size_t>> &potentialDistances,
      const int neighbour_x, const int neighbour_y);

  /*!
   * @brief gets distances
   */
  inline std::vector<std::pair<double, size_t>> &
  getPotentialDistancesSpeedField(
      const std::vector<size_t> &potentialSources,
      std::vector<std::pair<double, size_t>> &potentialDistances,
      const int neighbour_x, const int neighbour_y);

  inline double evaluateDistance(const int x1, const int y1, const int x2,
                                 const int y2) const {
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
  };
  inline double evaluateDistanceSpeedField(const int source_x,
                                           const int source_y,
                                           const int target_x,
                                           const int target_y) const {
    return sqrt((double)(source_x - target_x) * (source_x - target_x) +
                (source_y - target_y) * (source_y - target_y)) *
           sharedSpeedField_->get(target_x, target_y);
  };
  inline void createNewPivot(const int x, const int y, const int neighbour_x,
                             const int neighbour_y);

  void saveResults(const std::vector<point> &path,
                   const std::string &methodName);
  void saveImageWithPath(const std::vector<point> &path,
                         const std::string &methodName);
  void
  saveVisibilityBasedSolverImage(const std::unique_ptr<Field<double>> &gScore);

  /*!
   * @brief Insert to hashmap through a key.
   * @param [in] x position.
   * @param [in] y position.
   * @param [in] lightSource_enum number of the light source.
   * @param [in] value of visibility.
   */
  inline void insertIntoVisibilityHashMap(const size_t x, const size_t y,
                                          const size_t lightSource_num,
                                          const double value) {
    visibilityHashMap_[y + nx_ * x + ny_ * nx_ * (lightSource_num + 0)] = value;
    // visibilityHashMap_.try_emplace(y + nx_ * x + ny_ * nx_ * (lightSource_num
    // + 0), value);
  }

  /*!
   * @brief Return visibility value at a queried key.
   * @param [in] x position.
   * @param [in] y position.
   * @param [in] lightSource_enum number of the light source.
   */
  inline double VisibilityHashMapAt(const size_t x, const size_t y,
                                    const size_t lightSource_num) const {
    return visibilityHashMap_.at(y + nx_ * x +
                                 ny_ * nx_ * (lightSource_num + 0));
  };

  /*!
   * @brief Check if a key exists in hashmap.
   * @param [in] x position.
   * @param [in] y position.
   * @param [in] lightSource_enum number of the light source.
   */
  inline bool ExistsInVisibilityHashMap(const size_t x, const size_t y,
                                        const size_t lightSource_num) const {
    return visibilityHashMap_.count(y + nx_ * x +
                                    ny_ * nx_ * (lightSource_num + 0));
    // return visibilityHashMap_.contains(y + nx_ * x + ny_ * nx_ *
    // (lightSource_num + 0));
  };

  /*!
   * @brief Updates accessibility/visibility to a point using PDE advection.
   * @param [in] lightSourceNumber number of the lightsource whose visibility of
   * the point we are checking.
   * @param [in] lightSource_x x position of the lightsource.
   * @param [in] lightSource_y y position of the lightsource.
   * @param [in] x position of our queried pixel.
   * @param [in] y position of our queried pixel.
   */
  void updatePointVisibility(const size_t lightSourceNumber,
                             const int lightSource_x, const int lightSource_y,
                             const int x, const int y);

  std::shared_ptr<Field<double>> sharedVisibilityField_;
  std::shared_ptr<Field<double>> sharedSpeedField_;

  std::unique_ptr<Field<double>> gScore_;
  std::unique_ptr<Field<double>> fScore_;
  std::unique_ptr<Field<size_t>> cameFrom_;
  std::unique_ptr<Field<bool>> inOpenSet_;
  std::unique_ptr<Field<bool>> inClosedSet_;
  std::unique_ptr<Field<bool>> isUpdated_;

  std::shared_ptr<Config> sharedConfig_;
  std::unique_ptr<point[]> lightSources_;

  // Lightstrength, can be decreased. Can add later an alpha that has light
  // decay, enforcing adding a new pivot periodically.
  const double lightStrength_ = 1.0;
  int nb_of_iterations_ = 0;
  size_t nb_of_sources_ = 0;

  // Neighbours
  // [1 0; 0 1; -1 0; 0 -1; 1 1; -1 1; -1 -1; 1 -1] flattened out
  const int neighbours_[16] = {1, 0, 0,  1, -1, 0,  0, -1,
                               1, 1, -1, 1, -1, -1, 1, -1};
  const double neighbour_distances_[16] = {
      1,       1,       1,       1,       1,       1,       1,       1,
      sqrt(2), sqrt(2), sqrt(2), sqrt(2), sqrt(2), sqrt(2), sqrt(2), sqrt(2)};
  const int neighbours_4[8] = {1, 0, 0, 1, -1, 0, 0, -1};
  const double neighbour_distances_4[8] = {1, 1, 1, 1, 1, 1, 1, 1};

  const int neighbours_24[48] = {
      1,  0, 0,  1, -1, 0,  0,  -1, 1,  1,  -1, 1,  -1, -1, 1,  -1,
      2,  0, 0,  2, -2, 0,  0,  -2, 2,  1,  2,  2,  1,  2,  -1, 2,
      -2, 2, -2, 1, -2, -1, -2, -2, -1, -2, 1,  -2, 2,  -2, 2,  -1};
  // Dimensions.
  size_t ny_;
  size_t nx_;
  // Heap openSet_;
  std::unique_ptr<std::priority_queue<Node>> openSet_;

  double visibilityThreshold_ = 0.5;

  // Flat hash maps
  Map visibilityHashMap_;

  // Unique pointer to image holder
  std::unique_ptr<sf::Image> uniqueLoadedImage_;
};

} // namespace vbs
#endif // SOLVER_HPP
