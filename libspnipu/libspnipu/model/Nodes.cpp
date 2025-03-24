#include "libspnipu/model/Nodes.hpp"

#include <iostream>

#include "libspnipu/model/Visitor.hpp"

using namespace spnipu;

// ----------------- Node -----------------
template <NodeTraversalOrder Order>
void Node::walk(std::function<void(NodeRef)> visitor) {
  if constexpr (Order == NodeTraversalOrder::PreOrder) {
    visitor(this);
  }
  for (const NodeRef child : getChildren()) {
    child->walk<Order>(visitor);
  }
  if constexpr (Order == NodeTraversalOrder::PostOrder) {
    visitor(this);
  }
}

template void Node::walk<NodeTraversalOrder::PreOrder>(
    std::function<void(NodeRef)> visitor);
template void Node::walk<NodeTraversalOrder::PostOrder>(
    std::function<void(NodeRef)> visitor);
// ----------------- SumNode -----------------

void SumNode::dump(std::string intent) const {
  std::cout << "SumNode:" << std::endl;
  intent += "  ";
  for (size_t i = 0; i < children_.size(); ++i) {
    std::cout << intent << weights_[i] << " * ";
    children_[i]->dump(intent);
  }
}

void SumNode::accept(NodeVisitor* visitor) { visitor->visit(this); }

// ----------------- ProductNode -----------------
void ProductNode::dump(std::string intent) const {
  std::cout << "ProductNode:" << std::endl;
  intent += "  ";
  for (const NodeRef factor : children_) {
    std::cout << intent;
    factor->dump(intent);
  }
}

void ProductNode::accept(NodeVisitor* visitor) { visitor->visit(this); }

// ----------------- GaussianLeafNode -----------------
void GaussianLeafNode::dump(std::string intent) const {
  std::cout << "GaussianLeafNode (mean=" << mean_ << ", variance=" << variance_
            << ", scope=" << scope_ << ")" << std::endl;
}

const GaussianLeafNode::ChildContainerT& GaussianLeafNode::getChildren() const {
  static ChildContainerT empty;
  return empty;
}

void GaussianLeafNode::accept(NodeVisitor* visitor) { visitor->visit(this); }