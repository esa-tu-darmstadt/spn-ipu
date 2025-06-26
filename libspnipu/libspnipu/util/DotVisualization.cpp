#include "libspnipu/util/DotVisualization.hpp"

#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <typeinfo>
#include <unordered_set>
#include <vector>

#include "libspnipu/scheduling/PerformanceModel.hpp"

using namespace spnipu;

namespace {

// Helper to generate node IDs
std::string getNodeId(NodeRef node) {
  std::stringstream ss;
  ss << "node_" << node;
  return ss.str();
}

// Helper for node attributes
std::string getNodeAttributes(NodeRef node, unsigned proc,
                              PerformanceModel& model) {
  std::stringstream attrs;

  int compCost = model.getComputationCost(node, proc);

  // Set different shapes and colors based on node type
  if (dynamic_cast<SumNode*>(node)) {
    attrs << "shape=diamond, style=filled, fillcolor=lightblue, ";
    attrs << "label=\"Sum\\nCost=" << compCost << "\"";
  } else if (dynamic_cast<ProductNode*>(node)) {
    attrs << "shape=box, style=filled, fillcolor=lightgreen, ";
    attrs << "label=\"Product\\nCost=" << compCost << "\"";
  } else if (auto* gaussianNode = dynamic_cast<GaussianLeafNode*>(node)) {
    attrs << "shape=ellipse, style=filled, fillcolor=lightyellow, ";
    attrs << "label=\"Gaussian\\nCost=" << compCost
          << "\\nScope=" << gaussianNode->getScope() << "\"";
  } else {
    attrs << "shape=ellipse, ";
    attrs << "label=\"Unknown\"";
  }

  return attrs.str();
}

// Helper to generate edge definitions
void writeEdges(std::ofstream& file, NodeRef node) {
  const std::string nodeId = getNodeId(node);

  for (const NodeRef child : node->getChildren()) {
    const std::string childId = getNodeId(child);
    file << "  " << nodeId << " -> " << childId << ";" << std::endl;
  }
}

// Helper function for detailed BSP schedule visualization (individual nodes)
void plotBSPScheduleDetailed(std::ofstream& file, BSPSchedule& schedule,
                             PerformanceModel& model) {
  const SPN& spn = schedule.getSPN();
  const unsigned numSupersteps = schedule.getNumSupersteps();
  const unsigned numProcessors = schedule.getNumProcessors();

  // Create subgraphs for each superstep showing individual nodes
  for (unsigned superstep = 0; superstep < numSupersteps; ++superstep) {
    file << "  subgraph cluster_superstep_" << superstep << " {" << std::endl;
    file << "    label=\"Superstep " << superstep << "\";" << std::endl;
    file << "    style=filled;" << std::endl;
    file << "    color=lightgrey;" << std::endl;
    file << "    rank=same;" << std::endl;

    // Create subgraphs for each processor within this superstep
    for (unsigned proc = 0; proc < numProcessors; ++proc) {
      file << "    subgraph cluster_superstep_" << superstep << "_proc_" << proc
           << " {" << std::endl;
      file << "      label=\"Processor " << proc << "\";" << std::endl;
      file << "      style=filled;" << std::endl;
      file << "      color=white;" << std::endl;

      // Add nodes for this superstep and processor
      spn.getRoot()->walk(
          [&file, &schedule, superstep, proc, &model](NodeRef node) {
            if (schedule.isScheduled(node) &&
                schedule.getSuperstep(node) == superstep &&
                schedule.getProcessor(node) == proc) {
              file << "      " << getNodeId(node) << " ["
                   << getNodeAttributes(node, proc, model) << "];" << std::endl;
            }
          });

      file << "    }" << std::endl;  // End processor subgraph
    }

    file << "  }" << std::endl;  // End superstep subgraph
  }

  // Define edges between individual nodes
  std::unordered_set<NodeRef> visited;
  spn.getRoot()->walk([&file, &visited](NodeRef node) {
    if (visited.find(node) == visited.end()) {
      writeEdges(file, node);
      visited.insert(node);
    }
  });
}

// Helper function for simplified BSP schedule visualization (grid with one node
// per CPU per superstep)
void plotBSPScheduleSimplified(std::ofstream& file, BSPSchedule& schedule) {
  const SPN& spn = schedule.getSPN();
  const unsigned numSupersteps = schedule.getNumSupersteps();
  const unsigned numProcessors = schedule.getNumProcessors();

  // Create a grid structure: each superstep contains one node per processor
  for (unsigned superstep = 0; superstep < numSupersteps; ++superstep) {
    file << "  subgraph cluster_superstep_" << superstep << " {" << std::endl;
    file << "    label=\"Superstep " << superstep << "\";" << std::endl;
    file << "    style=filled;" << std::endl;
    file << "    color=lightgrey;" << std::endl;
    file << "    rank=same;" << std::endl;

    // Create one node per processor for this superstep
    for (auto& [proc, nodes] : schedule.getNodesOfSuperstep(superstep)) {
      // Count nodes scheduled on this processor in this superstep
      unsigned nodeCount = 0;
      std::vector<std::string> nodeTypes;

      spn.getRoot()->walk(
          [&schedule, superstep, proc, &nodeCount, &nodeTypes](NodeRef node) {
            if (schedule.isScheduled(node) &&
                schedule.getSuperstep(node) == superstep &&
                schedule.getProcessor(node) == proc) {
              nodeCount++;
              if (dynamic_cast<SumNode*>(node)) {
                nodeTypes.push_back("S");
              } else if (dynamic_cast<ProductNode*>(node)) {
                nodeTypes.push_back("P");
              } else if (dynamic_cast<GaussianLeafNode*>(node)) {
                nodeTypes.push_back("L");
              }
            }
          });

      // Create a node for this processor in this superstep
      file << "    proc_" << superstep << "_" << proc << " [" << std::endl;

      if (nodeCount > 0) {
        file << "      shape=box, style=filled, fillcolor=lightblue,"
             << std::endl;
        file << "      label=\"CPU " << proc << "\\n" << nodeCount << " nodes";

        // Show node types if space allows (limit to avoid cluttering)
        if (nodeTypes.size() <= 5) {
          file << "\\n";
          for (size_t i = 0; i < nodeTypes.size(); ++i) {
            if (i > 0) file << ",";
            file << nodeTypes[i];
          }
        }
        file << "\"" << std::endl;
      } else {
        file << "      shape=box, style=filled, fillcolor=lightgray,"
             << std::endl;
        file << "      label=\"CPU " << proc << "\\nidle\"" << std::endl;
      }

      file << "    ];" << std::endl;
    }

    file << "  }" << std::endl;  // End superstep subgraph
  }

  using ProcInSuperstep = std::pair<unsigned, unsigned>;
  using ProcInSuperstepEdge = std::pair<ProcInSuperstep, ProcInSuperstep>;
  std::map<ProcInSuperstepEdge, size_t> edges;

  for (unsigned superstep = 0; superstep < numSupersteps; ++superstep) {
    // Find parent-child relationships between consecutive supersteps
    for (auto& [parentProc, parentNodes] :
         schedule.getNodesOfSuperstep(superstep)) {
      for (NodeRef parent : parentNodes) {
        for (NodeRef child : parent->getChildren()) {
          unsigned childProc = schedule.getProcessor(child);
          unsigned childSuperstep = schedule.getSuperstep(child);

          if (childSuperstep == superstep) {
            // Only include edges between supersteps
            continue;
          }

          auto edge = std::make_pair(std::make_pair(superstep, parentProc),
                                     std::make_pair(childSuperstep, childProc));

          edges[edge]++;
        }
      }
    }
  }

  // Add edges between processors in consecutive supersteps
  for (const auto& [edge, count] : edges) {
    const auto& [source, target] = edge;
    file << "  proc_" << source.first << "_" << source.second << " -> "
         << "proc_" << target.first << "_" << target.second
         << " [style=dashed, color=gray, label=\"" << count << " comm\"];"
         << std::endl;
  }
}  // namespace

// Helper function for detailed partitioning visualization (individual nodes)
void plotPartitioningDetailed(std::ofstream& file, Partitioning& partitioning,
                              PerformanceModel& model) {
  const auto& partitions = partitioning.getPartitions();

  // Color palette for different partitions
  const std::vector<std::string> colors = {
      "lightblue", "lightgreen", "lightcoral", "lightyellow", "lightpink",
      "lightgray", "lightcyan",  "wheat",      "mistyrose",   "lavender"};

  size_t partitionIndex = 0;

  // Create subgraphs for each partition
  for (const auto& partition : partitions) {
    if (partition->empty()) {
      continue;  // Skip empty partitions
    }

    const std::string color = colors[partitionIndex % colors.size()];

    file << "  subgraph cluster_partition_" << partitionIndex << " {"
         << std::endl;
    file << "    label=\"Partition " << partitionIndex << " ("
         << partition->size() << " nodes)\";" << std::endl;
    file << "    style=filled;" << std::endl;
    file << "    color=" << color << ";" << std::endl;
    file << "    fillcolor=" << color << ";" << std::endl;
    file << "    alpha=0.3;" << std::endl;

    // Add nodes in this partition
    for (const NodeRef node : partition->getNodes()) {
      file << "    " << getNodeId(node) << " ["
           << getNodeAttributes(node, 0, model) << "];" << std::endl;
    }

    file << "  }" << std::endl;  // End partition subgraph
    ++partitionIndex;
  }

  // Define edges between all nodes
  std::unordered_set<NodeRef> visited;
  partitioning.getSPN().getRoot()->walk([&file, &visited](NodeRef node) {
    if (visited.find(node) == visited.end()) {
      writeEdges(file, node);
      visited.insert(node);
    }
  });
}

// Helper function for simplified partitioning visualization (partitions only)
void plotPartitioningSimplified(std::ofstream& file,
                                Partitioning& partitioning) {
  const auto& partitions = partitioning.getPartitions();

  // Color palette for different partitions
  const std::vector<std::string> colors = {
      "lightblue", "lightgreen", "lightcoral", "lightyellow", "lightpink",
      "lightgray", "lightcyan",  "wheat",      "mistyrose",   "lavender"};

  size_t partitionIndex = 0;
  std::unordered_map<PartitionRef, size_t> partitionToIndex;

  // Create simplified view showing only partition summaries
  for (const auto& partition : partitions) {
    if (partition->empty()) {
      continue;  // Skip empty partitions
    }

    partitionToIndex[partition.get()] = partitionIndex;
    const std::string color = colors[partitionIndex % colors.size()];

    file << "  partition_" << partitionIndex << " [" << std::endl;
    file << "    shape=box, style=filled, fillcolor=" << color << ","
         << std::endl;

    // Count different node types in this partition
    unsigned sumNodes = 0, productNodes = 0, leafNodes = 0;
    for (const NodeRef node : partition->getNodes()) {
      if (dynamic_cast<SumNode*>(node)) {
        sumNodes++;
      } else if (dynamic_cast<ProductNode*>(node)) {
        productNodes++;
      } else if (dynamic_cast<GaussianLeafNode*>(node)) {
        leafNodes++;
      }
    }

    file << "    label=\"Partition " << partitionIndex << "\\n"
         << partition->size() << " total nodes\\n"
         << sumNodes << " sum, " << productNodes << " product\\n"
         << leafNodes << " leaf nodes\"" << std::endl;
    file << "  ];" << std::endl;
    ++partitionIndex;
  }

  // Add edges between partitions using the Partitioning API
  std::set<std::pair<size_t, size_t>> addedEdges;  // Avoid duplicate edges

  for (const auto& partition : partitions) {
    if (partition->empty()) continue;

    auto sourceIt = partitionToIndex.find(partition.get());
    if (sourceIt == partitionToIndex.end()) continue;

    for (const auto& edge : partitioning.getOutgoingEdges(partition.get())) {
      auto targetIt = partitionToIndex.find(edge.target);
      if (targetIt == partitionToIndex.end()) continue;

      size_t sourceIdx = sourceIt->second;
      size_t targetIdx = targetIt->second;

      // Avoid duplicate edges
      if (addedEdges.find({sourceIdx, targetIdx}) == addedEdges.end()) {
        file << "  partition_" << sourceIdx << " -> partition_" << targetIdx
             << " [style=dashed, color=gray];" << std::endl;
        addedEdges.insert({sourceIdx, targetIdx});
      }
    }
  }
}

}  // anonymous namespace

namespace spnipu {

void plotSPNAsDot(SPN& spn, const std::filesystem::path& filename) {
  std::ofstream file(filename);
  PerformanceModel model(spn);
  file << "digraph SPN {" << std::endl;
  file << "  graph [rankdir=TB];" << std::endl;
  file << "  node [fontname=\"Arial\"];" << std::endl;

  // Keep track of visited nodes to avoid duplicates
  std::unordered_set<NodeRef> visited;

  // Define nodes
  spn.getRoot()->walk([&file, &visited, &model](NodeRef node) {
    // Only process each node once
    if (visited.find(node) == visited.end()) {
      file << "  " << getNodeId(node) << " ["
           << getNodeAttributes(node, 0, model) << "];" << std::endl;
      visited.insert(node);
    }
  });

  // Reset visited set for edge generation
  visited.clear();

  // Define edges
  spn.getRoot()->walk([&file, &visited](NodeRef node) {
    if (visited.find(node) == visited.end()) {
      writeEdges(file, node);
      visited.insert(node);
    }
  });

  file << "}" << std::endl;
}

void plotBSPScheduleAsDot(BSPSchedule& schedule,
                          const std::filesystem::path& filename,
                          bool showIndividualNodes) {
  PerformanceModel model(schedule.getSPN());
  std::ofstream file(filename);

  file << "digraph BSPSchedule {" << std::endl;
  file << "  graph [rankdir=TB];" << std::endl;
  file << "  node [fontname=\"Arial\"];" << std::endl;

  // Call the appropriate visualization function
  if (showIndividualNodes) {
    plotBSPScheduleDetailed(file, schedule, model);
  } else {
    plotBSPScheduleSimplified(file, schedule);
  }

  file << "}" << std::endl;
}

void plotPartitioningAsDot(Partitioning& partitioning,
                           const std::filesystem::path& filename,
                           bool showIndividualNodes) {
  PerformanceModel model(partitioning.getSPN());
  std::ofstream file(filename);

  file << "digraph Partitioning {" << std::endl;
  file << "  graph [rankdir=TB, compound=true];" << std::endl;
  file << "  node [fontname=\"Arial\"];" << std::endl;

  // Call the appropriate visualization function
  if (showIndividualNodes) {
    plotPartitioningDetailed(file, partitioning, model);
  } else {
    plotPartitioningSimplified(file, partitioning);
  }

  file << "}" << std::endl;
}


}  // namespace spnipu