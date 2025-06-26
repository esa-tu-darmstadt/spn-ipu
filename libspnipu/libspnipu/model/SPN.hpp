#pragma once

#include <list>
#include <memory>
#include <string>
#include <unordered_map>

#include "libspnipu/model/Nodes.hpp"

namespace spnipu {

class SPN {
  NodeRef root_;
  std::list<std::unique_ptr<Node>> nodes_;
  std::string name_ = "Unknown SPN";

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

  // SPN metadata
  const std::string& getName() const { return name_; }
  void setName(const std::string& name) { name_ = name; }

  // Node statistics
  std::unordered_map<std::string, unsigned> getNodeTypeDistribution() const {
    std::unordered_map<std::string, unsigned> distribution;
    
    walk([&distribution](NodeRef node) {
      if (dynamic_cast<SumNode*>(node)) {
        distribution["sum"]++;
      } else if (dynamic_cast<ProductNode*>(node)) {
        distribution["product"]++;
      } else if (dynamic_cast<GaussianLeafNode*>(node)) {
        distribution["gaussian"]++;
      } else {
        distribution["unknown"]++;
      }
    });
    
    return distribution;
  }

  unsigned getTotalNodeCount() const {
    return nodes_.size();
  }
};

}  // namespace spnipu