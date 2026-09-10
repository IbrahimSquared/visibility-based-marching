#include "solver/visibility_cache.hpp"

#include <bit>
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>
#include <unordered_map>

namespace {
void require(const bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

bool sameBits(const double a, const double b) {
  return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b);
}

void checkLimits() {
  vbm::VisibilityCache cache;
  cache.reset(1);
  constexpr auto sentinel = std::numeric_limits<std::uint32_t>::max();
  double value = 17;
  for (const std::size_t source : {std::size_t(sentinel),
                                  std::numeric_limits<std::size_t>::max()}) {
    bool rejected = false;
    try {
      cache.tryGet(0, source, value);
    } catch (const std::length_error &) {
      rejected = true;
    }
    require(rejected && value == 17 && cache.size() == 0,
            "Invalid source lookup must not hit an empty record");
    rejected = false;
    try {
      cache.set(0, source, 1);
    } catch (const std::length_error &) {
      rejected = true;
    }
    require(rejected && cache.size() == 0,
            "Invalid source insertion must not mutate the cache");
  }
  cache.set(0, sentinel - 1, 0.75);
  require(cache.tryGet(0, sentinel - 1, value) && value == 0.75,
          "Largest supported source must work");
}

void checkReferenceParity() {
  vbm::VisibilityCache cache;
  std::mt19937_64 random(731);
  // Alternate grid sizes on the same cache to exercise retained capacity and
  // reset. Force multiple sources per cell and overflow-vector reallocations.
  for (const std::size_t cells : {1, 97, 7, 257, 1}) {
    cache.reset(cells);
    require(cache.size() == 0 && cache.cellCount() == cells,
            "Reset must remove every cached value");
    std::unordered_map<std::uint64_t, double> reference;
    for (int iteration = 0; iteration < 12000; ++iteration) {
      const auto cell = random() % cells;
      const auto source = random() % 31;
      const auto key = (source << 32) | cell;
      double value = 123;
      const auto found = reference.find(key);
      const bool hit = cache.tryGet(cell, source, value);
      require(hit == (found != reference.end()), "Hit/miss mismatch");
      require(hit ? sameBits(value, found->second) : value == 123,
              "Lookup value mismatch or miss modified output");

      // Include cached zero, negative zero, infinity and NaN. Visibility zero
      // must be distinguishable from absence, and storage must preserve bits.
      constexpr double special[] = {
          0.0, -0.0, std::numeric_limits<double>::infinity(),
          std::numeric_limits<double>::quiet_NaN()};
      const double inserted = iteration % 5 == 0
                                  ? special[iteration % 4]
                                  : double(random() % 1000000) / 1000000;
      cache.set(cell, source, inserted);
      reference[key] = inserted;
      require(cache.size() == reference.size(), "Entry count mismatch");
    }
    for (const auto &[key, expected] : reference) {
      double value = 0;
      require(cache.tryGet(key & 0xffffffffULL, key >> 32, value) &&
                  sameBits(value, expected),
              "Stored value lost during updates or vector growth");
    }
    const auto storage = cache.storageBytes();
    cache.reset(cells);
    require(cache.storageBytes() == storage, "Reset should reuse capacity");
    for (const auto &[key, ignored] : reference) {
      double value = 0;
      require(!cache.tryGet(key & 0xffffffffULL, key >> 32, value),
              "Reset retained a stale source/value");
    }
  }
  cache.reset(0);
  require(cache.size() == 0 && cache.cellCount() == 0,
          "Empty reset must be supported");
}
} // namespace

int main() {
  try {
    checkLimits();
    checkReferenceParity();
    std::cout << "Visibility cache reference, reset and boundary checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
