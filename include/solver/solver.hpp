#ifndef SOLVER_HPP
#define SOLVER_HPP

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
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

struct point {
  int x, y;
};

class Solver {
public:
  using Map = ska::flat_hash_map<size_t, double>;

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
  struct CandidateSources {
    std::array<size_t, 8> values{};
    std::uint8_t size = 0;

    inline void clear() noexcept { size = 0; }

    inline void addUnique(const size_t source) noexcept {
      for (std::uint8_t i = 0; i < size; ++i) {
        if (values[i] == source) {
          return;
        }
      }
      assert(size < values.size());
      if (size < values.size()) {
        values[size++] = source;
      }
    }
  };

  struct PotentialDistance {
    double distance = std::numeric_limits<double>::infinity();
    size_t source = 0;
  };

  void reset();
  void reconstructPath(const Node &current, const std::string &methodName);
  inline int indexAt(const size_t x, const size_t y) const {
    return x + y * nx_;
  };
  inline point coordinatesAt(const size_t index) const {
    const int x = index % nx_;
    const int y = (index) / nx_;
    return {x, y};
  }

  /*!
   * @brief queues unique sources from updated neighbors of queried cell.
   */
  inline void queuePotentialSources(CandidateSources &potentialSources,
                                    const int neighbour_x,
                                    const int neighbour_y) const;

  /*!
   * @brief returns best visible source/distance candidate for VStar.
   */
  inline PotentialDistance
  getPotentialDistance(const CandidateSources &potentialSources,
                       const int neighbour_x, const int neighbour_y);

  /*!
   * @brief returns best visible source/distance candidate for speed-field VBM.
   */
  inline PotentialDistance
  getPotentialDistanceSpeedField(const CandidateSources &potentialSources,
                                 const int neighbour_x, const int neighbour_y);

  /*!
   * @brief returns best source/distance candidate for plain distance function.
   */
  inline PotentialDistance
  getPotentialDistanceFunction(const CandidateSources &potentialSources,
                               const int neighbour_x,
                               const int neighbour_y) const;

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
                   const std::string &methodName) const;
  void saveImageWithPath(const std::vector<point> &path,
                         const std::string &methodName) const;
  void saveVisibilityBasedSolverImage(const Field<double> &gScore) const;
  void saveDistanceFunctionImage(const Field<double> &gScore) const;

  /*!
   * @brief Updates accessibility/visibility to a point using PDE advection.
   * @param [in] lightSourceNumber number of the lightsource whose visibility of
   * the point we are checking.
   * @param [in] lightSource_x x position of the lightsource.
   * @param [in] lightSource_y y position of the lightsource.
   * @param [in] x position of our queried pixel.
   * @param [in] y position of our queried pixel.
   * @return computed or cached visibility value for (x, y, lightSourceNumber).
   */
  double updatePointVisibility(const size_t lightSourceNumber,
                               const int lightSource_x, const int lightSource_y,
                               const int x, const int y);

  // Collision-free row-major key for the logical tuple
  // (x, y, lightSourceNumber). This is a stored map key, not merely a bucket
  // hash, so collisions would alias distinct visibility values.
  inline size_t hashFunction(const int x, const int y,
                             const size_t lightSourceNumber) const noexcept {
    assert(x >= 0);
    assert(y >= 0);
    assert(static_cast<size_t>(x) < nx_);
    assert(static_cast<size_t>(y) < ny_);

    return static_cast<size_t>(x) +
           nx_ * (static_cast<size_t>(y) + ny_ * lightSourceNumber);
  }

  /*!
   * @brief Checks if requested cell is in grid
   * @param [in] x x position of the cell
   * @param [in] y y position of the cell
   * @return true if cell is in grid, false otherwise
   */
  inline bool isValid(const size_t x, const size_t y) const {
    return ((x < nx_) && (y < ny_));
  }

  std::shared_ptr<Field<double>> sharedVisibilityField_;
  std::shared_ptr<Field<double>> sharedSpeedField_;

  Field<double> gScore_;
  Field<double> fScore_;
  Field<size_t> cameFrom_;
  Field<bool> inOpenSet_;
  Field<bool> updated_;

  std::shared_ptr<Config> sharedConfig_;
  std::vector<point> lightSources_;

  // Lightstrength, can be decreased. Can add later an alpha that has light
  // decay, enforcing adding a new pivot periodically.
  const double lightStrength_ = 1.0;
  int nb_of_iterations_ = 0;

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
