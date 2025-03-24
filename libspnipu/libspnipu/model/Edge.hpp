#pragma once

#include <cstddef>
#include <tuple>

namespace spnipu {
class Node;
using NodeRef = Node*;

/// References a directed edge in an SPN
class EdgeRef {
 public:
  EdgeRef(NodeRef source, NodeRef target) : source_(source), target_(target) {}

  NodeRef getSource() const { return source_; }
  NodeRef getTarget() const { return target_; }

  bool operator==(const EdgeRef& other) const {
    return source_ == other.source_ && target_ == other.target_;
  }

  bool operator!=(const EdgeRef& other) const { return !(*this == other); }

  bool operator<(const EdgeRef& other) const {
    return std::tie(source_, target_) < std::tie(other.source_, other.target_);
  }

 private:
  NodeRef source_;
  NodeRef target_;
};

}  // namespace spnipu

namespace std {
template <class Key>
struct hash;
template <>
struct hash<spnipu::EdgeRef> {
  std::size_t operator()(const spnipu::EdgeRef& edge) const {
    size_t src = (size_t)edge.getSource();
    size_t tgt = (size_t)edge.getTarget();
    return src ^ (tgt + 0x9e3779b9 + (src << 6) + (src >> 2));
  }
};
}  // namespace std