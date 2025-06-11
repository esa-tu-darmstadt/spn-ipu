#pragma once

#include <list>

#include "libspnipu/model/Nodes.hpp"

namespace spnipu {

class SPN {
  NodeRef root_;

  std::list<std::unique_ptr<Node>> nodes_;

 public:
  const NodeRef getRoot() const { return root_; }

  template <typename T, typename... Args>
  NodeRef createNode(Args &&...args) {
    nodes_.emplace_back(std::make_unique<T>(std::forward<Args>(args)...));
    return nodes_.back().get();
  }

  void setRoot(NodeRef root) { root_ = root; }
  void clearNodes() { nodes_.clear(); }

  void dump() const { root_->dump(); }

  const auto &getNodes() const { return nodes_; }

  template <NodeTraversalOrder Order = NodeTraversalOrder::PreOrder>
  void walk(std::function<void(NodeRef)> visitor) const {
    root_->walk<Order>(visitor);
  }

  unsigned getNumFeatures() const {
    unsigned numFeatures = 0;
    walk([&numFeatures](NodeRef node) {
      if (auto leaf = dynamic_cast<LeafNode *>(node)) {
        numFeatures = std::max(numFeatures, leaf->getScope());
      }
    });
    return numFeatures + 1;
  }
};

}  // namespace spnipu