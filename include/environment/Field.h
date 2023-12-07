#ifndef FIELD_H
#define FIELD_H

#include <cstddef>
#include <memory>

namespace vbs {

template <typename T>
class Field {
 public:
  using size_t = std::size_t;
  Field() : nx_(0), ny_(0), size_(0), data_(nullptr) {}

  explicit Field(size_t nx, size_t ny, T default_value)
      : nx_(nx), ny_(ny), size_(nx * ny) {
    data_ = std::make_unique<T[]>(size_);
    for (size_t i = 0; i < size_; ++i) {
      data_[i] = default_value;
    }
  }

  void set(const size_t x, const size_t y, const T value) {
    data_[x + y * nx_] = value;
  }
  const T get(const size_t x, const size_t y) const {
    return data_[x + y * nx_];
  }
  const size_t nx() const { return nx_; }
  const size_t ny() const { return ny_; }

  // copy constructor
  Field(const Field& other)
      : nx_(other.nx_), ny_(other.ny_), size_(other.size_) {
    data_ = std::make_unique<T[]>(size_);
    for (size_t i = 0; i < size_; ++i) {
      data_[i] = other.data_[i];
    }
  }
  // copy assignment
  Field& operator=(const Field& other) {
    if (this != &other) {
      nx_ = other.nx_;
      ny_ = other.ny_;
      size_ = other.size_;
      data_ = std::make_unique<T[]>(size_);
      for (size_t i = 0; i < size_; ++i) {
        data_[i] = other.data_[i];
      }
    }
    return *this;
  }

  // move constructor
  Field(Field&& other) noexcept
      : nx_(other.nx_), ny_(other.ny_), size_(other.size_) {
    data_ = std::move(other.data_);
  }

  // move assignment
  Field& operator=(Field&& other) noexcept {
    if (this != &other) {
      nx_ = other.nx_;
      ny_ = other.ny_;
      size_ = other.size_;
      data_ = std::move(other.data_);
    }
    return *this;
  }

  ~Field() = default;

 private:
  size_t nx_;
  size_t ny_;
  size_t size_;
  std::unique_ptr<T[]> data_;
};

}  // namespace vbs

#endif  // FIELD_H