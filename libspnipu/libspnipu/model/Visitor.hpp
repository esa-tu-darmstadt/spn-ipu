#pragma once

#include "libspnipu/model/Nodes.hpp"

namespace spnipu {
struct NodeVisitor {
  virtual ~NodeVisitor() = default;
  void visit(NodeRef node) { node->accept(this); }

  virtual void visit(ProductNode* node) = 0;
  virtual void visit(SumNode* node) = 0;
  virtual void visit(GaussianLeafNode* node) = 0;
};

}  // namespace spnipu