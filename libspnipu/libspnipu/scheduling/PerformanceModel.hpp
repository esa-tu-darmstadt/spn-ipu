#pragma once

#include "libspnipu/model/SPN.hpp"
namespace spnipu {

class PerformanceModel {
 public:
  struct NodeCostVisitor;

  PerformanceModel(SPN &spn);
  ~PerformanceModel();

  int getComputationCost(NodeRef node, unsigned proc) const;
  int getCommunicationCost() const;

 private:
  SPN &spn_;
  std::unique_ptr<NodeCostVisitor> nodeVostVisitor;
};
}  // namespace spnipu