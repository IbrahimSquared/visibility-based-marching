#ifndef SOLVER_H
#define SOLVER_H

#include "environment/environment.h"
#include "flat_hash_map/flat_hash_map.hpp"

#include <unordered_map>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <iostream>

namespace vbs {

struct Node {
  int x, y;
  double f;
  bool operator<(const Node& other) const {
      return f > other.f;
  }
};

struct BucketStats {
  size_t occupied = 0;
  size_t total_collisions = 0;
  size_t max_collisions = 0;

  BucketStats(std::unordered_map<size_t, double> const& c) {
    for(auto bucket = c.bucket_count(); bucket--;) {
      auto bucket_size = c.bucket_size(bucket);
      occupied += bucket_size > 0;
      if(bucket_size > 1) {
        auto collisions = bucket_size - 1;
        total_collisions += collisions;
        max_collisions = std::max(max_collisions, collisions);
      }
    }
  }

  double avg_collisions() const {
    return occupied ? static_cast<double>(total_collisions) / occupied : 0;
  }

  friend std::ostream& operator<<(std::ostream& s, BucketStats const& b) {
    return s
      << "used buckets: " << b.occupied
      << "; total collisions: " << b.total_collisions
      << "; max collisions in a bucket: " << b.max_collisions
      << "; avg collisions per bucket: " << b.avg_collisions();
  }
};

class solver {
 public:

  using Map = ska::flat_hash_map<size_t, double>;
  // using Map = std::unordered_map<size_t, double>; // , std::hash<size_t>>;
  using point = std::pair<size_t, size_t>;

  solver(environment& env);

  // Deconstructor
  ~solver() = default;

  void vStarSearch();
  void aStarSearch();
  void visibilityBasedSolver();
  void computeDistanceFunction();

  // Get global iterator (number of iterations that had to be completed)
  inline int getNbOfIterations() const { return nb_of_iterations_; };
  // Get hashmap load factor - measure of badness of a hashmap
  inline double getLoadFactor() const { return visibilityHashMap_.load_factor(); };

private:
    std::vector<std::pair<size_t, size_t>> getPointsBetween(int x1, int y1, int x2, int y2) const;

    void reset();
    void reconstructPath(const Node& current, const std::string& methodName);
    inline int indexAt(const size_t x, const size_t y) const { return x * ncols_ + y; };
    inline point coordinatesAt(size_t index) const {
        size_t y = index % ncols_;
        size_t x = (index - y) / ncols_;
        return {x, y};
    }

    static size_t xorshift(const size_t& n, int i){
      return n^(n>>i);
    }
    static size_t hash(const size_t& n){
      size_t p = 0x5555555555555555ull; // pattern of alternating 0 and 1
      size_t c = 17316035218449499591ull;// random uneven integer constant; 
      return c*xorshift(p*xorshift(n,32),32);
    }

    /*!
    * @brief queues sources
    */
    inline std::vector<size_t>& queuePotentialSources(std::vector<size_t>& potentialSources, const int neighbour_x, const int neighbour_y) const;

    /*!
    * @brief gets distances
    */
    inline std::vector<std::pair<double, size_t>>& getPotentialDistances(const std::vector<size_t>& potentialSources, std::vector<std::pair<double, size_t>>& potentialDistances, const int neighbour_x, const int neighbour_y);

    /*!
    * @brief gets distances
    */
    inline std::vector<std::pair<double, size_t>>& getPotentialDistancesSpeedField(const std::vector<size_t>& potentialSources, std::vector<std::pair<double, size_t>>& potentialDistances, const int neighbour_x, const int neighbour_y);


    /*
    cityblock distance between (x1,y1) and (x2,y2) is abs(x1-x2)
    + abs(y1-y2).  The chessboard distance is max(abs(x1-x2),
    abs(y1-y2)).  The quasi-Euclidean distance is:
        abs(x1-x2) + (sqrt(2)-1)*abs(y1-y2),  if abs(x1-x2) > abs(y1-y2)
        (sqrt(2)-1)*abs(x1-x2) + abs(y1-y2),  otherwise
    */

    inline double evaluateDistance(const int x1, const int y1, const int x2, const int y2) const { return sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) ); };
    inline double evaluateDistanceSpeedField(const int source_x, const int source_y, const int target_x, const int target_y) const { 
      return sqrt((double)(source_x-target_x) * (source_x-target_x) + (source_y-target_y) * (source_y-target_y))
        * sharedSpeedField_(target_x, target_y);
    };

    /*
    // Canberra (|x1 - x2| / (|x1| + |x2|)) + (|y1 - y2| / (|y1| + |y2|))
    inline double evaluateDistance(const int x1, const int y1, const int x2, const int y2) const { return double(abs(x1-x2))/(x1 + x2) + double(abs(y1-y2))/(y1+y2) ; };
    inline double evaluateDistanceSpeedField(const int source_x, const int source_y, const int target_x, const int target_y) const { 
      return double(abs(source_x-target_x))/(source_x + target_x) + double(abs(source_y-target_y))/(source_y+target_y)
        * sharedSpeedField_(target_x, target_y);
    };
    */

    /*
    // rewrite the two equations above but to compute the Minkowsi distance with p = 3 instead
    inline double evaluateDistance(const int x1, const int y1, const int x2, const int y2) const { return pow( pow(abs(x1-x2), 3) + pow(abs(y1-y2), 3), 1.0/3.0); };
    inline double evaluateDistanceSpeedField(const int source_x, const int source_y, const int target_x, const int target_y) const {
      return pow( pow(abs(source_x-target_x), 3) + pow(abs(source_y-target_y), 3), 1.0/3.0)
        * sharedSpeedField_(target_x, target_y);
    };
    */

    /*
    inline double evaluateDistance(const int x1, const int y1, const int x2, const int y2) const { return abs(y1 - y2) + abs(x1 - x2); };
    inline double evaluateDistanceSpeedField(const int source_x, const int source_y, const int target_x, const int target_y) const { 
      return (abs(target_x - source_x) + abs(target_y - source_y)) * sharedSpeedField_(target_x, target_y);
    };
    */

    /*
    inline double evaluateDistance(const int x1, const int y1, const int x2, const int y2) const { return std::max(abs(x1-x2), abs(y1-y2)); };
    inline double evaluateDistanceSpeedField(const int source_x, const int source_y, const int target_x, const int target_y) const { 
      return std::max(abs(source_x-target_x), abs(source_y-target_y)) * sharedSpeedField_(target_x, target_y);
    };
    */

    /*
    inline double evaluateDistance(const int x1, const int y1, const int x2, const int y2) const { 
      if (abs(x1-x2) > abs(y1-y2)) {
        return abs(x1-x2) + (sqrt(2)-1)*abs(y1-y2);
      } else {
        return (sqrt(2)-1)*abs(x1-x2) + abs(y1-y2);
      }
    };
    inline double evaluateDistanceSpeedField(const int source_x, const int source_y, const int target_x, const int target_y) const { 
      if (abs(source_x-target_x) > abs(source_y-target_y)) {
        return ( abs(source_x-target_x) + (sqrt(2)-1)*abs(source_y-target_y) ) * sharedSpeedField_(target_x, target_y);
      } else {
        return ( (sqrt(2)-1)*abs(source_x-target_x) + abs(source_y-target_y) ) * sharedSpeedField_(target_x, target_y);
      }
    };
    */

    inline void createNewPivot(const int x, const int y, const int neighbour_x, const int neighbour_y);

    void saveResults(const std::vector<point>& path, const std::string& methodName);
    void saveImageWithPath(const std::vector<point>& path, const std::string& methodName);

    /*!
    * @brief Insert to hashmap through a key.
    * @param [in] x position.
    * @param [in] y position.
    * @param [in] lightSource_enum number of the light source.
    * @param [in] value of visibility.
    */
    inline void insertIntoVisibilityHashMap(const size_t x, const size_t y, const size_t lightSource_num, const double value) {
        visibilityHashMap_[y + ncols_ * x + nrows_ * ncols_ * (lightSource_num + 0)] = value;
        // visibilityHashMap_.try_emplace(y + ncols_ * x + nrows_ * ncols_ * (lightSource_num + 0), value);
    }

    /*!
    * @brief Return visibility value at a queried key.
    * @param [in] x position.
    * @param [in] y position.
    * @param [in] lightSource_enum number of the light source.
    */
    inline double VisibilityHashMapAt(const size_t x, const size_t y, const size_t lightSource_num) const { 
      return visibilityHashMap_.at(y + ncols_ * x + nrows_ * ncols_ * (lightSource_num + 0)); 
    };

    /*!
    * @brief Check if a key exists in hashmap.
    * @param [in] x position.
    * @param [in] y position.
    * @param [in] lightSource_enum number of the light source.
    */
    inline bool ExistsInVisibilityHashMap(const size_t x, const size_t y, const size_t lightSource_num) const { 
      return visibilityHashMap_.count(y + ncols_ * x + nrows_ * ncols_ * (lightSource_num + 0));
      // return visibilityHashMap_.contains(y + ncols_ * x + nrows_ * ncols_ * (lightSource_num + 0)); 
    };

    /*!
    * @brief Updates accessibility/visibility to a point using PDE advection.
    * @param [in] lightSourceNumber number of the lightsource whose visibility of the point we are checking.
    * @param [in] lightSource_x x position of the lightsource.
    * @param [in] lightSource_y y position of the lightsource.
    * @param [in] x position of our queried pixel.
    * @param [in] y position of our queried pixel.
    */
    void updatePointVisibility(const size_t lightSourceNumber, const int lightSource_x, const int lightSource_y, const int x, const int y);

    Field<double, 1> sharedVisibilityField_;
    Field<double, 1> sharedSpeedField_;
    std::shared_ptr<Config> sharedConfig_;

    Field<bool, 0> inOpenSet_;
    Field<bool, 0> inClosedSet_;
    Field<double, 0> gScore_;
    Field<double, 0> fScore_;
    Field<size_t, 0> cameFrom_;
    Field<bool, 0> isUpdated_;
    std::unique_ptr<point[]> lightSources_;

    // Lightstrength, can be decreased. Can add later an alpha that has light decay, enforcing adding a new pivot periodically.
    const double lightStrength_ = 1.0;
    int nb_of_iterations_ = 0;
    size_t nb_of_sources_ = 0;

    // ((x * ncols + y) * max_num + index) % (nrows * ncols * max_num)

    // Neighbours
    // [1 0; 0 1; -1 0; 0 -1; 1 1; -1 1; -1 -1; 1 -1] flattened out
    const int neighbours_[16] = {1, 0, 0, 1, -1, 0, 0, -1, 1, 1, -1, 1, -1, -1, 1, -1};
    const double neighbour_distances_[16] = {1, 1, 1, 1, 1, 1, 1, 1,
      sqrt(2), sqrt(2), sqrt(2), sqrt(2), sqrt(2), sqrt(2), sqrt(2), sqrt(2)};
    const int neighbours_4[8] = {1, 0, 0, 1, -1, 0, 0, -1};
    const double neighbour_distances_4[8] = {1, 1, 1, 1, 1, 1, 1, 1};

    const int neighbours_24[48] = {1, 0, 0, 1, -1, 0, 0, -1,
      1, 1, -1, 1, -1, -1, 1, -1,
      2, 0, 0, 2, -2, 0, 0, -2,
      2, 1, 2, 2, 1, 2, -1, 2, -2, 2, -2, 1, -2, -1, -2, -2, -1, -2, 1, -2, 2, -2, 2, -1};
    // Dimensions.
    size_t nrows_; size_t ncols_;
    // Heap openSet_;
    std::unique_ptr<std::priority_queue<Node>> openSet_;

    double visibilityThreshold_= 0.5;

    // Flat hash maps
    Map visibilityHashMap_;

    // Unique pointer to image holder
    std::unique_ptr<sf::Image> uniqueLoadedImage_;

    static const size_t max_num = 1610612741;
};

} // namespace vbs
#endif