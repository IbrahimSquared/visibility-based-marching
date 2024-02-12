#ifndef SOLVER_HPP
#define SOLVER_HPP

#include <cmath>
#include <queue>
#include <vector>

#include "environment/environment.hpp"
#include "flat_hash_map/flat_hash_map.hpp"

namespace vbm {

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
  inline point coordinatesAt(const size_t index) const {
    const size_t x = index % nx_;
    const size_t y = (index) / nx_;
    return {x, y};
  }

  /*!
   * @brief queues sources
   */
  inline void queuePotentialSources(std::vector<size_t> &potentialSources,
                                    const int neighbour_x,
                                    const int neighbour_y) const;

  /*!
   * @brief gets distances
   */
  inline void getPotentialDistances(
      const std::vector<size_t> &potentialSources,
      std::vector<std::pair<double, size_t>> &potentialDistances,
      const int neighbour_x, const int neighbour_y);

  /*!
   * @brief gets distances
   */
  inline void getPotentialDistancesSpeedField(
      const std::vector<size_t> &potentialSources,
      std::vector<std::pair<double, size_t>> &potentialDistances,
      const int neighbour_x, const int neighbour_y);

  inline const double evaluateDistance(const int x1, const int y1, const int x2,
                                       const int y2) const {
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
  };
  inline const double evaluateDistanceSpeedField(const int source_x,
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
  void saveVisibilityBasedSolverImage(const Field<double> &gScore);
  void saveDistanceFunctionImage(const Field<double> &gScore);

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

  inline const int hashFunction(const int x, const int y,
                                const int lightSourceNumber) const {
    const auto key = y + nx_ * x + nx_ * ny_ * lightSourceNumber;
    return key;
  }

  std::shared_ptr<Field<double>> sharedVisibilityField_;
  std::shared_ptr<Field<double>> sharedSpeedField_;

  Field<double> gScore_;
  Field<double> fScore_;
  Field<size_t> cameFrom_;
  Field<bool> inOpenSet_;
  Field<bool> isUpdated_;

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

} // namespace vbm
#endif // SOLVER_HPP
