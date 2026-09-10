#ifndef VBM_VISIBILITY_CACHE_HPP
#define VBM_VISIBILITY_CACHE_HPP

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace vbm {

// One inline record per cell, with additional sources linked by indices into
// a shared vector. All queried sources are retained, including visibility zero.
// Lookup/set require cell < cellCount(). No references survive an insertion.
class VisibilityCache {
public:
  void reset(const std::size_t cellCount) {
    cells_.clear();
    overflow_.clear();
    entryCount_ = 0;
    cells_.resize(cellCount);
  }

  // A miss leaves visibility unchanged. Return values are copied out so that
  // recursive visibility evaluations can safely grow the overflow vector.
  bool tryGet(const std::size_t cell, const std::size_t source,
              double &visibility) const {
    assert(cell < cells_.size());
    const auto sourceId = checkedSource(source);
    const auto &first = cells_[cell];
    if (first.source == sourceId) {
      visibility = first.visibility;
      return true;
    }
    for (auto next = first.next; next != none; next = overflow_[next].next) {
      const auto &entry = overflow_[next];
      if (entry.source == sourceId) {
        visibility = entry.visibility;
        return true;
      }
    }
    return false;
  }

  void set(const std::size_t cell, const std::size_t source,
           const double visibility) {
    assert(cell < cells_.size());
    const auto sourceId = checkedSource(source);
    auto &first = cells_[cell];
    if (first.source == sourceId) {
      first.visibility = visibility;
      return;
    }
    if (first.source == none) {
      first.source = sourceId;
      first.visibility = visibility;
      ++entryCount_;
      return;
    }
    for (auto next = first.next; next != none; next = overflow_[next].next) {
      auto &entry = overflow_[next];
      if (entry.source == sourceId) {
        entry.visibility = visibility;
        return;
      }
    }
    if (overflow_.size() >= none) {
      throw std::length_error("Visibility cache overflow index exceeds uint32_t");
    }
    const auto next = static_cast<std::uint32_t>(overflow_.size());
    overflow_.push_back({visibility, sourceId, first.next});
    first.next = next;
    ++entryCount_;
  }

  std::size_t size() const noexcept { return entryCount_; }
  std::size_t cellCount() const noexcept { return cells_.size(); }
  std::size_t storageBytes() const noexcept {
    return (cells_.capacity() + overflow_.capacity()) * sizeof(Entry);
  }

private:
  static constexpr auto none = std::numeric_limits<std::uint32_t>::max();

  struct Entry {
    double visibility = 0;
    std::uint32_t source = none;
    std::uint32_t next = none;
  };
  static_assert(sizeof(Entry) == 16);

  static std::uint32_t checkedSource(const std::size_t source) {
    // Reject the empty-record sentinel before either lookup or insertion.
    if (source >= none) {
      throw std::length_error("Visibility cache source ID exceeds uint32_t");
    }
    return static_cast<std::uint32_t>(source);
  }

  std::vector<Entry> cells_;
  std::vector<Entry> overflow_;
  std::size_t entryCount_ = 0;
};

} // namespace vbm

#endif // VBM_VISIBILITY_CACHE_HPP
