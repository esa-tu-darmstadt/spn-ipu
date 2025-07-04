#pragma once

#include <filesystem>
#include <optional>
#include <set>
#include <unordered_map>
#include <unordered_set>

#include "libspnipu/model/Edge.hpp"
#include "libspnipu/model/Nodes.hpp"
#include "libspnipu/model/SPN.hpp"
namespace spnipu {

class BSPSchedule {
  template <typename T>
  using NodeAttr = std::unordered_map<NodeRef, T>;
  template <typename T>
  using SuperstepAttr = std::unordered_map<NodeRef, T>;

  const bool expensiveChecks_ = true;

  struct BSPSuperstep {
    // Maps each processor to the nodes that are scheduled on it in this
    // superstep
    std::unordered_map<unsigned, std::unordered_set<NodeRef>> nodes;

    // Edges to nodes that must execute before this superstep, i.e., this
    // superstep depends on.
    std::unordered_set<EdgeRef> predecessorEdges;

    // Edges to nodes that must execute after this superstep, i.e., this
    // superstep produces results for.
    std::unordered_set<EdgeRef> successorEdges;
  };

 public:
  BSPSchedule(const SPN& spn) : spn_(spn) {}

  /// Returns the superstep in which a node is scheduled.
  unsigned getSuperstep(NodeRef node) const {
    return nodeToSuperstep_.at(node);
  }
  /// Returns the superstep in which a node is scheduled.
  unsigned getProcessor(NodeRef node) const { return nodeToProc_.at(node); }

  /// Returns a mapping of processors to the nodes that are scheduled on them in
  /// the given superstep.
  const std::unordered_map<unsigned, std::unordered_set<NodeRef>>&
  getNodesOfSuperstep(unsigned superstep) const {
    static std::unordered_map<unsigned, std::unordered_set<NodeRef>> empty;
    if (superstep >= supersteps_.size()) return empty;
    return supersteps_.at(superstep).nodes;
  }

  /// Returns true if a node is scheduled.
  bool isScheduled(NodeRef node) const {
    return nodeToSuperstep_.find(node) != nodeToSuperstep_.end();
  }

  /// Schedule a node in a superstep on a processor.
  void scheduleNode(NodeRef node, unsigned superstep, unsigned proc) {
    if (locked_) {
      throw std::runtime_error(
          "Cannot schedule node after locking the schedule");
    }
    nodeToSuperstep_[node] = superstep;
    nodeToProc_[node] = proc;

    if (supersteps_.size() <= superstep) {
      supersteps_.resize(superstep + 1);
    }
    supersteps_[superstep].nodes[proc].insert(node);

    numSupersteps_ = std::max(numSupersteps_, superstep + 1);
    numProcessors_ = std::max(numProcessors_, proc + 1);
  }

  /// Dumps information about the schedule to the console.
  void dump() const;

  /// Validates the schedule and throws an exception if it is invalid.
  void validate() const;

  /// Locks the schedule so that no more nodes can be scheduled. Computes the
  /// incoming and outgoing edges of each superstep.
  void lock();

  /// Returns true if the schedule is locked.
  bool isLocked() const { return locked_; }

  const SPN& getSPN() const { return spn_; }

  // Creates a schedule that runs all nodes in a single superstep on a single
  // processor. Useful for debugging.
  static BSPSchedule singleSuperstep(SPN& spn);

  unsigned getNumSupersteps() const { return numSupersteps_; }

  unsigned getNumProcessors() const { return numProcessors_; }

  // Returns the predecessor edges of a superstep, i.e. edges to nodes that
  // must execute before this superstep. Only valid after the schedule is
  // locked.
  const std::unordered_set<EdgeRef>& getPredecessorEdges(
      unsigned superstep) const;

  // Returns the successor edges of a superstep, i.e. edges to nodes that
  // must execute after this superstep. Only valid after the schedule is locked.
  const std::unordered_set<EdgeRef>& getSuccessorEdges(
      unsigned superstep) const;

  /// Returns a new schedule identical to this but without empty supersteps.
  BSPSchedule withoutEmptySupersteps() const;

  /// Returns a description of the scheduling algorithm used to generate this
  /// schedule.
  const std::string& getSchedulingAlgorithmDescription() const {
    return schedulingAlgorithmDescr;
  }

  /// Sets the description of the scheduling algorithm used to generate this
  /// schedule.
  void setSchedulingAlgorithmDescription(const std::string& description) {
    schedulingAlgorithmDescr = description;
  }

  /// Returns a description of the algorithm used to partition the SPN
  /// before scheduling, if partitioning was used.
  const std::optional<std::string>& getPartitioningAlgorithmDescription()
      const {
    return partitioningAlgorithmDescr;
  }

  /// Sets the description of the algorithm used to partition the SPN
  /// before scheduling, if partitioning was used.
  void setPartitioningAlgorithmDescription(
      const std::optional<std::string>& description) {
    partitioningAlgorithmDescr = description;
  }

 private:
  // Maps each node to the superstep it is scheduled in
  NodeAttr<unsigned> nodeToSuperstep_;
  // Maps each node to the processor it is scheduled on
  NodeAttr<unsigned> nodeToProc_;

  std::vector<BSPSuperstep> supersteps_;

  unsigned numSupersteps_ = 0;
  unsigned numProcessors_ = 0;

  bool locked_ = false;

  const SPN& spn_;

  std::string schedulingAlgorithmDescr = "Unknown";
  std::optional<std::string> partitioningAlgorithmDescr = "Unknown";
};

}  // namespace spnipu