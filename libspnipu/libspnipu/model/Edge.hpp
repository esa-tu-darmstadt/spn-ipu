#pragma once

#include <cstddef>
#include <tuple>

namespace spnipu {
class Node;
using NodeRef = Node*;

/// References a directed edge in an SPN
class EdgeRef {
 public:
  EdgeRef(NodeRef predecessor, NodeRef successor)
      : predecessor_(predecessor), successor_(successor) {}

  NodeRef getPredecessor() const { return predecessor_; }
  NodeRef getSuccessor() const { return successor_; }

  bool operator==(const EdgeRef& other) const {
    return predecessor_ == other.predecessor_ && successor_ == other.successor_;
  }

  bool operator!=(const EdgeRef& other) const { return !(*this == other); }

  bool operator<(const EdgeRef& other) const {
    return std::tie(predecessor_, successor_) <
           std::tie(other.predecessor_, other.successor_);
  }

 private:
  NodeRef predecessor_;
  NodeRef successor_;
};

}  // namespace spnipu

namespace std {
template <class Key>
struct hash;
template <>
struct hash<spnipu::EdgeRef> {
  std::size_t operator()(const spnipu::EdgeRef& edge) const {
    size_t src = (size_t)edge.getPredecessor();
    size_t tgt = (size_t)edge.getSuccessor();
    return src ^ (tgt + 0x9e3779b9 + (src << 6) + (src >> 2));
  }
};
}  // namespace std