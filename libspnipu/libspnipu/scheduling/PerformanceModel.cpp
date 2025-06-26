#include "libspnipu/scheduling/PerformanceModel.hpp"

#include "libspnipu/model/Visitor.hpp"

using namespace spnipu;

struct PerformanceModel::NodeCostVisitor : NodeVisitor {
  virtual int getCost() const = 0;
};

namespace {

struct NonLogNodeCostVisitor : PerformanceModel::NodeCostVisitor {
  int result = 0;
  int getCost() const override { return result; }

  void visit(ProductNode* node) final {
    // product *= child => 1 FLOP per child
    result = node->getChildren().size();
  }
  void visit(SumNode* node) final {
    // sum += weight * child => 2 FLOPs per child
    result = node->getChildren().size() * 2;
  }
  void visit(GaussianLeafNode* node) final {
    // requires around 6 FLOPs
    result = 6;
  }
};
}  // namespace

PerformanceModel::PerformanceModel(const SPN& spn) : spn_(spn) {
  nodeVostVisitor = std::make_unique<NonLogNodeCostVisitor>();
}

PerformanceModel::~PerformanceModel() = default;

int PerformanceModel::getComputationCost(NodeRef node, unsigned proc) const {
  // The computation cost is the number of cycles required to compute the node
  node->accept(nodeVostVisitor.get());
  return nodeVostVisitor->getCost();
}

int PerformanceModel::getCommunicationCost() const {
  // The communication cost is the number of cycles
  return 1;
}

int PerformanceModel::getComputationCost(PartitionRef partition, unsigned proc) const {
  if (!partition) {
    return 0;
  }
  
  int totalCost = 0;
  for (NodeRef node : partition->getNodes()) {
    totalCost += getComputationCost(node, proc);
  }
  return totalCost;
}

int PerformanceModel::getCommunicationCost(const PartitionEdge &edge) const {
  // Communication cost is the single edge cost times the number of edges between the partitions
  int edgeCount = 0;
  
  // Count edges from source partition nodes to target partition nodes
  for (NodeRef sourceNode : edge.source->getNodes()) {
    for (NodeRef child : sourceNode->getChildren()) {
      // Check if this child is in the target partition
      for (NodeRef targetNode : edge.target->getNodes()) {
        if (child == targetNode) {
          edgeCount++;
          break;
        }
      }
    }
  }
  
  return getCommunicationCost() * edgeCount;
}