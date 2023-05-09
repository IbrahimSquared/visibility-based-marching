#ifndef CONFIG_PARSER_H
#define CONFIG_PARSER_H

#include <string>
#include <vector>

namespace vbs {

using size_t = std::size_t;

struct Config {
    int mode = 1;
    size_t ncols = 100;
    size_t nrows = 100;
    int nb_of_obstacles = 10;
    size_t minWidth = 10;
    size_t maxWidth = 20;
    size_t minHeight = 10;
    size_t maxHeight = 20;
    float speedValue = 2.0;
    bool randomSeed = true;
    int seedValue = 0;
    std::string imagePath = "C:\\...";
    std::vector<int> initialFrontline;
    int target_x;
    int target_y;
    float visibilityThreshold = 0.5;
    bool timer = true;
    bool greedy = true;
    bool visibilityBasedSolver = true;
    bool expandInObstacles = true;
    bool vstar = true;
    bool astar = true;
    bool distanceFunction = true;
    bool saveResults = true;
    bool saveCameFrom = true;
    bool saveGScore = true;
    bool saveFScore = true;
    bool saveLightSources = true;
    bool saveVisibilityField = true;
    bool saveSpeedField = true;
    bool savePath = true;
    bool saveDistanceFunction = true;
    bool silent = false;
};

class ConfigParser {
public:
  ConfigParser(){};
  bool parse(const std::string& filename);
  inline const Config& getConfig() const { return config_; };
  // Deconstructor
  ~ConfigParser() = default;

private:
    Config config_;
    std::vector<int> parseVectorString(const std::string& str);
};

} // namespace vbs
#endif // CONFIG_PARSER_H