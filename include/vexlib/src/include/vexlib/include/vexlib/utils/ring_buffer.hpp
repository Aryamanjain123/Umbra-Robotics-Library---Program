#pragma once
#include <vector>

namespace vexlib::utils {

template <typename T>
class RingBuffer {
 public:
  explicit RingBuffer(size_t cap) : data_(cap) {}
  
  void push(const T& v) {
    if (data_.empty()) return;
    data_[head_] = v;
    head_ = (head_ + 1) % data_.size();
    size_ = std::min(size_ + 1, data_.size());
  }

  size_t size() const { return size_; }
  size_t capacity() const { return data_.size(); }

  const T& operator[](size_t i) const {
    return data_[(head_ + data_.size() - size_ + i) % data_.size()];
  }

 private:
  std::vector<T> data_;
  size_t head_{0};
  size_t size_{0};
};

}  // namespace vexlib::utils
