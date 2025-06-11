#include "libspnipu/scheduling/BSPSchedule.hpp"

#include <fstream>
#include <iostream>

using namespace spnipu;

BSPSchedule::BSPSchedule(SPN& spn) : spn_(spn) {}

void BSPSchedule::dump() const {
  for (const auto& [node, superstep] : nodeToSuperstep_) {
    std::cout << "Node " << node << " is scheduled in superstep " << superstep
              << std::endl;
  }
}

void BSPSchedule::validate() const {
  spn_.getRoot()->walk([this](NodeRef node) {
    if (nodeToSuperstep_.find(node) == nodeToSuperstep_.end()) {
      throw std::runtime_error(
          "Node is not scheduled (no entry in nodeToSuperstep_).");
    }
    if (nodeToProc_.find(node) == nodeToProc_.end()) {
      throw std::runtime_error(
          "Node is not scheduled (no entry in nodeToProc_).");
    }
  });

  for (const auto& [node, superstep] : nodeToSuperstep_) {
    for (const NodeRef child : node->getChildren()) {
      unsigned childSuperstep = nodeToSuperstep_.at(child);
      if (childSuperstep > superstep) {
        throw std::runtime_error("Node is scheduled after child.");
      } else if (childSuperstep == superstep) {
        // The child is scheduled in the same superstep as the parent. Make sure
        // that the parent is scheduled on the same processor as the child.
        if (nodeToProc_.at(node) != nodeToProc_.at(child)) {
          throw std::runtime_error(
              "Node is scheduled in the same superstep as "
              "child, but on different processors.");
        }
      }
    }
  }
}

BSPSchedule BSPSchedule::singleSuperstep(SPN& spn) {
  BSPSchedule schedule(spn);
  unsigned superstep = 0;
  unsigned proc = 0;
  spn.getRoot()->walk([&schedule, &superstep, &proc](NodeRef node) {
    schedule.scheduleNode(node, superstep, proc);
  });
  return schedule;
}

void BSPSchedule::lock() {
  if (locked_) return;
  locked_ = true;

  // Calculate incoming and outgoing edges for each superstep
  for (unsigned i = 0; i < supersteps_.size(); ++i) {
    BSPSuperstep& ss = supersteps_.at(i);

    // Calculate outgoing edges
    for (const auto [node, superstep] : nodeToSuperstep_) {
      if (superstep == i) {
        // This node is scheduled in this superstep, so it cannot have outgoing
        // edges in this superstep
        continue;
      }
      for (NodeRef child : node->getChildren()) {
        if (nodeToSuperstep_.at(child) == i) {
          ss.outgoingEdges.insert({child, node});
        }
      }
    }

    // Calculate incoming edges
    for (auto& [proc, nodes] : ss.nodes) {
      for (const NodeRef node : nodes) {
        for (NodeRef child : node->getChildren()) {
          if (nodeToSuperstep_.at(child) != i) {
            ss.incomingEdges.insert({child, node});
          }
        }
      }
    }
  }
}

const std::unordered_set<EdgeRef>& BSPSchedule::getIncomingEdges(
    unsigned superstep) const {
  if (!locked_) {
    throw std::runtime_error("Schedule is not locked.");
  }
  return supersteps_.at(superstep).incomingEdges;
}
const std::unordered_set<EdgeRef>& BSPSchedule::getOutgoingEdges(
    unsigned superstep) const {
  if (!locked_) {
    throw std::runtime_error("Schedule is not locked.");
  }
  return supersteps_.at(superstep).outgoingEdges;
}

BSPSchedule BSPSchedule::withoutEmptySupersteps() const {
  BSPSchedule newSchedule(spn_);
  unsigned shiftedSuperstep = 0;
  for (unsigned s = 0; s < numSupersteps_; s++) {
    if (getNodesOfSuperstep(s).size() > 0) {
      for (auto& [proc, nodes] : getNodesOfSuperstep(s)) {
        for (NodeRef node : nodes) {
          newSchedule.scheduleNode(node, shiftedSuperstep, proc);
        }
      }
      shiftedSuperstep++;
    }
  }
  newSchedule.lock();
  return newSchedule;
}