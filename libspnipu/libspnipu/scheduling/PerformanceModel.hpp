#pragma once

#include "libspnipu/model/SPN.hpp"
#include "libspnipu/model/Partitioning.hpp"

namespace spnipu {

class PerformanceModel {
 public:
  struct NodeCostVisitor;

  PerformanceModel(const SPN &spn);
  ~PerformanceModel();

  int getComputationCost(NodeRef node, unsigned proc) const;
  int getCommunicationCost() const;
  
  int getComputationCost(PartitionRef partition, unsigned proc) const;
  int getCommunicationCost(const PartitionEdge &edge) const;

 private:
  const SPN &spn_;
  std::unique_ptr<NodeCostVisitor> nodeVostVisitor;
};
}  // namespace spnipu