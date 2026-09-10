// Built only by compare_visibility_cache.py, against isolated source copies
// whose Solver fields are made public for exact state comparison.
#include "solver/solver.hpp"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <sys/resource.h>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;

template <typename T> void append(std::vector<char> &bytes, const T &value) {
  const auto *start = reinterpret_cast<const char *>(&value);
  bytes.insert(bytes.end(), start, start + sizeof(value));
}

std::vector<char> snapshot(const vbm::Solver &solver, const bool success) {
  std::vector<char> bytes;
  append(bytes, success);
  append(bytes, solver.nb_of_iterations_);
  append(bytes, solver.lightSources_.size());
  for (const auto source : solver.lightSources_) {
    append(bytes, source.x);
    append(bytes, source.y);
  }
  for (std::size_t y = 0; y < solver.ny_; ++y) {
    for (std::size_t x = 0; x < solver.nx_; ++x) {
      append(bytes, solver.gScore_(x, y));
      append(bytes, solver.fScore_(x, y));
      append(bytes, solver.cameFrom_(x, y));
      append(bytes, solver.updated_(x, y));
      append(bytes, solver.inOpenSet_(x, y));
    }
  }
  return bytes;
}

bool solve(vbm::Solver &solver, const std::string &method) {
  if (method == "vbm") return solver.visibilityBasedSolver();
  if (method == "vstar") return solver.vStarSearch();
  if (method == "astar") return solver.aStarSearch();
  if (method == "distance") return solver.computeDistanceFunction();
  throw std::invalid_argument("Unknown solver method");
}
} // namespace

int main(int argc, char **argv) {
  try {
    if (argc != 12) throw std::invalid_argument("Expected 11 probe arguments");
    const std::string kind = argv[1], method = argv[7];
    const int width = std::stoi(argv[2]), height = std::stoi(argv[3]);
    const int seed = std::stoi(argv[4]), repeats = std::stoi(argv[5]);
    const auto setupStart = Clock::now();
    vbm::Config config;
    config.ncols = width;
    config.nrows = height;
    config.nb_of_obstacles = 0;
    config.silent = true;
    config.saveResults = false;
    config.randomSeed = false;
    config.visibilityThreshold = std::stof(argv[8]);
    config.expandInObstacles = std::stoi(argv[9]);
    config.speedValue = std::stof(argv[10]);
    config.greedy = std::stoi(argv[11]);
    vbm::Environment environment(config);
    auto field = environment.getVisibilityField();
    std::mt19937 random(seed);
    if (kind == "sparse" || kind == "dense") {
      const int count = kind == "sparse" ? 15 : 120;
      const int divisor = kind == "sparse" ? 7 : 15;
      for (int i = 0; i < count; ++i) {
        const int x = random() % width, y = random() % height;
        const int w = 1 + random() % std::max(2, width / divisor);
        const int h = 1 + random() % std::max(2, height / divisor);
        for (int yy = y; yy < std::min(height, y + h); ++yy)
          for (int xx = x; xx < std::min(width, x + w); ++xx)
            field->set(xx, yy, 0);
      }
    } else if (kind == "noise") {
      for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x)
          if (random() % 100 < 20) field->set(x, y, 0);
    } else if (kind == "maze") {
      const int step = std::max(4, width / 20);
      int wall = 0;
      for (int x = step; x < width - 1; x += step, ++wall)
        for (int y = 0; y < height; ++y)
          if ((wall % 2 == 0 && y < height - step) ||
              (wall % 2 && y >= step)) field->set(x, y, 0);
    } else if (kind == "single") {
      field->set(width / 2, height / 2, 0);
    } else if (kind == "diagonal") {
      field->set(1, 0, 0);
      field->set(0, 1, 0);
    } else if (kind.starts_with("image:")) {
      sf::Image image;
      if (!image.loadFromFile(kind.substr(6))) return 2;
      const auto size = image.getSize();
      for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x) {
          const auto pixel = image.getPixel(std::size_t(x) * size.x / width,
                                           std::size_t(y) * size.y / height);
          field->set(x, y, pixel.r + pixel.g + pixel.b >= 384 ? 1 : 0);
        }
    }
    int sourceX = -1, sourceY = -1, goalX = 0, goalY = 0;
    int nearest = std::numeric_limits<int>::max();
    for (int y = 0; y < height; ++y)
      for (int x = 0; x < width; ++x) {
        if (field->get(x, y) == 0) {
          environment.getSpeedField()->set(x, y, config.speedValue);
          continue;
        }
        const int distance = std::abs(x - width / 2) + std::abs(y - height / 2);
        if (distance < nearest) {
          nearest = distance;
          sourceX = x;
          sourceY = y;
        }
        goalX = x;
        goalY = y;
      }
    if (sourceX < 0) throw std::runtime_error("Probe map has no free cell");
    if (kind == "diagonal") sourceX = sourceY = 0;
    auto settings = environment.getConfig();
    settings->initialFrontline = {sourceX, height - 1 - sourceY};
    settings->target_x = goalX;
    settings->target_y = height - 1 - goalY;
    if (kind == "multi")
      settings->initialFrontline = {width / 4, height - 1 - height / 4,
                                   3 * width / 4, height - 1 - 3 * height / 4};
    vbm::Solver solver(environment);
    const double setupMs = std::chrono::duration<double, std::milli>(
                               Clock::now() - setupStart).count();
    std::vector<double> times;
    std::vector<char> expected;
    double coldMs = 0;
    for (int iteration = -1; iteration < repeats; ++iteration) {
      const auto start = Clock::now();
      const bool success = solve(solver, method == "sequence" ? "vbm" : method);
      const double elapsed = std::chrono::duration<double, std::milli>(
                                 Clock::now() - start).count();
      if (iteration == -1) coldMs = elapsed;
      else times.push_back(elapsed);
      auto current = snapshot(solver, success);
      if (method == "sequence") {
        // Exercise reset across methods on the same instance. These extra
        // calls are parity-only and excluded from the reported VBM timing.
        for (const std::string next : {"vstar", "astar", "distance", "vbm"}) {
          auto state = snapshot(solver, solve(solver, next));
          current.insert(current.end(), state.begin(), state.end());
        }
      }
      if (iteration >= 0 && current != expected)
        throw std::runtime_error("Nondeterministic solver output across resets");
      expected = std::move(current);
    }
    std::ofstream output(argv[6], std::ios::binary);
    output.write(expected.data(), expected.size());
    if (!output) throw std::runtime_error("Unable to save probe output");
    rusage usage{};
    getrusage(RUSAGE_SELF, &usage);
#ifdef VBM_LEGACY_CACHE
    const auto entries = solver.visibilityHashMap_.size();
#else
    const auto entries = solver.visibilityCache_.size();
#endif
    std::cout << std::setprecision(12)
              << "{\"cold_ms\":" << coldMs << ",\"setup_ms\":" << setupMs
              << ",\"rss_kib\":" << usage.ru_maxrss
              << ",\"sources\":" << solver.lightSources_.size()
              << ",\"cache_entries\":" << entries << ",\"times_ms\":[";
    for (std::size_t i = 0; i < times.size(); ++i)
      std::cout << (i ? "," : "") << times[i];
    std::cout << "]}\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
