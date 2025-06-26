#pragma once

#include <set>
#include <stdexcept>

#include "libspnipu/model/SPN.hpp"

namespace spnipu {

class Partition;
using PartitionRef = Partition *;

class Partitioning;

struct PartitionEdge {
  PartitionRef predecessor;
  PartitionRef successor;

  PartitionEdge(PartitionRef source, PartitionRef target)
      : predecessor(source), successor(target) {}

  bool operator==(const PartitionEdge &other) const {
    return predecessor == other.predecessor && successor == other.successor;
  }
};

class Partition {
  friend class Partitioning;
  std::list<NodeRef> nodes_;

  std::list<PartitionEdge>
      successorEdges_;  // Partitions that execute after this one
  std::list<PartitionEdge>
      predecessorEdges_;  // Partitions that execute before this one

 public:
  /// Constructs an empty partition
  Partition() = default;

  /// Constructs a partition with the given nodes
  Partition(std::initializer_list<NodeRef> nodes) : nodes_(nodes) {}

  /// Returns the nodes in this partition
  const std::list<NodeRef> &getNodes() const { return nodes_; }

  /// Returns the number of nodes in this partition
  size_t size() const { return nodes_.size(); }

  /// Returns true if this partition is empty
  bool empty() const { return nodes_.empty(); }
};

class Partitioning {
  const SPN &spn_;
  std::list<std::unique_ptr<Partition>> partitions_;
  std::unordered_map<NodeRef, PartitionRef> nodeToPartition_;
  std::string partitionAlgorithmDescr_ = "Unknown";

  bool locked_ = false;

 public:
  /// Constructs an empty partitioning
  Partitioning(const SPN &spn) : spn_(spn) {}

  /// Constructs a partitioning in which each node of the SPN is in its own
  /// partition
  static Partitioning createOneToOnePartitioning(const SPN &spn) {
    Partitioning partitioning(spn);
    for (const auto &node : spn.getNodes()) {
      PartitionRef partition = partitioning.createPartition<Partition>();
      partitioning.addNodeToPartition(node.get(), partition);
    }
    partitioning.setPartitioningAlgorithmDescription(
        "No partitioning (one-to-one)");
    return partitioning;
  }

  const SPN &getSPN() const { return spn_; }

  template <typename T>
  PartitionRef createPartition() {
    partitions_.emplace_back(std::make_unique<T>());
    return partitions_.back().get();
  }

  void addNodeToPartition(NodeRef node, PartitionRef partition) {
    if (node == nullptr) {
      throw std::invalid_argument("Node cannot be invalid");
    }

    if (partition == nullptr) {
      throw std::invalid_argument("Partition cannot be null");
    }

    if (nodeToPartition_.find(node) != nodeToPartition_.end()) {
      throw std::runtime_error("Node is already in a partition");
    }

    partition->nodes_.push_back(node);
    nodeToPartition_[node] = partition;
  }

  PartitionRef getPartition(NodeRef node) const {
    auto it = nodeToPartition_.find(node);
    if (it == nodeToPartition_.end()) {
      return nullptr;  // Node is not in any partition
    }
    return it->second;
  }

  void lock() {
    if (locked_) {
      throw std::runtime_error("Partitioning is already locked");
    }
    locked_ = true;
    calculateEdges();
  }

  /// Returns partitions that must execute after the given partition
  const auto &getSuccessorEdges(PartitionRef partition) const {
    if (!partition) {
      throw std::invalid_argument("Partition cannot be null");
    }
    if (!locked_) {
      throw std::runtime_error("Partitioning must be locked to access edges");
    }
    return partition->successorEdges_;
  }

  /// Returns partitions that must execute before the given partition
  const auto &getPredecessorEdges(PartitionRef partition) const {
    if (!partition) {
      throw std::invalid_argument("Partition cannot be null");
    }
    if (!locked_) {
      throw std::runtime_error("Partitioning must be locked to access edges");
    }
    return partition->predecessorEdges_;
  }

  const auto &getPartitions() const { return partitions_; }

  const std::string &getPartitioningAlgorithmDescription() const {
    return partitionAlgorithmDescr_;
  }

  void setPartitioningAlgorithmDescription(const std::string &description) {
    partitionAlgorithmDescr_ = description;
  }

 private:
  void calculateEdges() {
    for (const auto &partition : partitions_) {
      for (const auto &node : partition->nodes_) {
        PartitionRef parentPartition = getPartition(node);
        if (!parentPartition) continue;

        for (const auto &child : node->getChildren()) {
          PartitionRef childPartition = getPartition(child);
          if (!childPartition || childPartition == parentPartition) continue;

          // Child partition must execute before parent partition
          childPartition->successorEdges_.emplace_back(childPartition,
                                                       parentPartition);
          parentPartition->predecessorEdges_.emplace_back(childPartition,
                                                          parentPartition);
        }
      }
    }
  }
};
}  // namespace spnipu