#pragma once

#include <boost/container/small_vector.hpp>
#include <functional>
#include <vector>

namespace spnipu {
struct NodeVisitor;
class Node;
using NodeRef = Node*;

enum class NodeTraversalOrder {
  // PreOrder: Visit the node before its children
  PreOrder,
  // PostOrder: Visit the node after its children
  PostOrder
};

class Node {
 public:
  using ChildContainerT = boost::container::small_vector<NodeRef, 4>;

  virtual ~Node() = default;

  void dump() const {
    // Using a function overload instead of a default argument so that the dump
    // function can be called from the debugger easier
    dump("");
  }
  virtual void dump(std::string intent) const = 0;
  virtual const ChildContainerT& getChildren() const = 0;

  template <NodeTraversalOrder Order = NodeTraversalOrder::PreOrder>
  void walk(std::function<void(NodeRef)> visitor);
  virtual void accept(NodeVisitor* visitor) = 0;
};

class LeafNode : public Node {
 public:
  virtual unsigned getScope() const = 0;
};

class SumNode : public Node {
  ChildContainerT children_;
  boost::container::small_vector<double, 4> weights_;

 public:
  SumNode() = default;
  SumNode(std::vector<std::pair<NodeRef, double>> children)
      : children_{}, weights_{} {
    for (const auto& [node, weight] : children) {
      children_.push_back(node);
      weights_.push_back(weight);
    }
  }
  ~SumNode() override = default;

  void addSummand(NodeRef node, double weight) {
    children_.push_back(node);
    weights_.push_back(weight);
  }

  void dump(std::string intent = "") const override;
  const ChildContainerT& getChildren() const override { return children_; }

  double getWeight(unsigned i) const { return weights_[i]; }

  void accept(NodeVisitor* visitor) override;
};

class ProductNode : public Node {
  ChildContainerT children_;

 public:
  ProductNode() = default;
  ProductNode(std::vector<NodeRef> children)
      : children_(children.begin(), children.end()) {}

  ~ProductNode() override = default;

  void addFactor(NodeRef node) { children_.push_back(node); }

  void dump(std::string intent = "") const override;

  const ChildContainerT& getChildren() const override { return children_; }

  void accept(NodeVisitor* visitor) override;
};

class GaussianLeafNode : public LeafNode {
  double mean_;
  double variance_;
  unsigned scope_;

 public:
  GaussianLeafNode(double mean, double variance, unsigned scope)
      : mean_(mean), variance_(variance), scope_(scope) {}
  ~GaussianLeafNode() override = default;

  void dump(std::string intent = "") const override;

  const ChildContainerT& getChildren() const override;

  void accept(NodeVisitor* visitor) override;

  unsigned getScope() const override { return scope_; }

  double getMean() const { return mean_; }
  double getVariance() const { return variance_; }
};
}  // namespace spnipu
