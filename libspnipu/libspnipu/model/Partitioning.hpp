#pragma once

#include <set>
#include <stdexcept>

#include "libspnipu/model/SPN.hpp"

namespace spnipu {

class Partition;
using PartitionRef = Partition *;

class Partitioning;

struct PartitionEdge {
  PartitionRef source;
  PartitionRef target;

  PartitionEdge(PartitionRef source, PartitionRef target)
      : source(source), target(target) {}

  bool operator==(const PartitionEdge &other) const {
    return source == other.source && target == other.target;
  }
};

class Partition {
  friend class Partitioning;
  std::list<NodeRef> nodes_;

  std::list<PartitionEdge> outgoingEdges_;
  std::list<PartitionEdge> incomingEdges_;

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

  const auto &getOutgoingEdges(PartitionRef partition) const {
    if (!partition) {
      throw std::invalid_argument("Partition cannot be null");
    }
    if (!locked_) {
      throw std::runtime_error("Partitioning must be locked to access edges");
    }
    return partition->outgoingEdges_;
  }

  const auto &getIncomingEdges(PartitionRef partition) const {
    if (!partition) {
      throw std::invalid_argument("Partition cannot be null");
    }
    if (!locked_) {
      throw std::runtime_error("Partitioning must be locked to access edges");
    }
    return partition->incomingEdges_;
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
        PartitionRef sourcePartition = getPartition(node);
        if (!sourcePartition) continue;

        for (const auto &child : node->getChildren()) {
          PartitionRef targetPartition = getPartition(child);
          if (!targetPartition || targetPartition == sourcePartition) continue;

          sourcePartition->outgoingEdges_.emplace_back(sourcePartition,
                                                       targetPartition);
          targetPartition->incomingEdges_.emplace_back(sourcePartition,
                                                       targetPartition);
        }
      }
    }
  }
};
}  // namespace spnipu