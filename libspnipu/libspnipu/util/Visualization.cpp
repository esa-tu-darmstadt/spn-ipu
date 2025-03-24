#include "libspnipu/util/Visualization.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <typeinfo>
#include <unordered_set>

namespace spnipu {

std::string DotVisualizer::getNodeId(NodeRef node) {
  // Use the pointer value as a unique ID
  std::stringstream ss;
  ss << "node_" << node;
  return ss.str();
}

std::string DotVisualizer::getNodeAttributes(NodeRef node) {
  std::stringstream attrs;

  // Set different shapes and colors based on node type
  if (dynamic_cast<SumNode*>(node)) {
    attrs << "shape=diamond, style=filled, fillcolor=lightblue, ";
    attrs << "label=\"Sum\\n";

    // Add weights if available
    const auto* sumNode = static_cast<SumNode*>(node);
    const auto& children = sumNode->getChildren();
    // Note: We can't easily get weights here due to class design,
    // but could be added in a future enhancement

    attrs << "\"";
  } else if (dynamic_cast<ProductNode*>(node)) {
    attrs << "shape=box, style=filled, fillcolor=lightgreen, ";
    attrs << "label=\"Product\"";
  } else if (auto* gaussianNode = dynamic_cast<GaussianLeafNode*>(node)) {
    attrs << "shape=ellipse, style=filled, fillcolor=lightyellow, ";
    attrs << "label=\"Gaussian\\nScope=" << gaussianNode->getScope() << "\"";
  } else {
    attrs << "shape=ellipse, ";
    attrs << "label=\"Unknown\"";
  }

  return attrs.str();
}

void DotVisualizer::writeEdges(std::ofstream& file, NodeRef node) {
  const std::string nodeId = getNodeId(node);

  for (const NodeRef child : node->getChildren()) {
    const std::string childId = getNodeId(child);
    file << "  " << nodeId << " -> " << childId << ";" << std::endl;
  }
}

void DotVisualizer::plotSPN(SPN& spn, const std::filesystem::path& filename) {
  std::ofstream file(filename);
  file << "digraph SPN {" << std::endl;
  file << "  graph [rankdir=TB];" << std::endl;
  file << "  node [fontname=\"Arial\"];" << std::endl;

  // Keep track of visited nodes to avoid duplicates
  std::unordered_set<NodeRef> visited;

  // Define nodes
  spn.getRoot()->walk([&file, &visited](NodeRef node) {
    // Only process each node once
    if (visited.find(node) == visited.end()) {
      file << "  " << getNodeId(node) << " [" << getNodeAttributes(node) << "];"
           << std::endl;
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

void DotVisualizer::plotBSPSchedule(BSPSchedule& schedule,
                                    const std::filesystem::path& filename) {
  std::ofstream file(filename);
  file << "digraph BSPSchedule {" << std::endl;
  file << "  graph [rankdir=TB];" << std::endl;
  file << "  node [fontname=\"Arial\"];" << std::endl;

  const SPN& spn = schedule.getSPN();
  const unsigned numSupersteps = schedule.getNumSupersteps();
  const unsigned numProcessors = schedule.getNumProcessors();

  // Create subgraphs for each superstep
  for (unsigned superstep = 0; superstep < numSupersteps; ++superstep) {
    file << "  subgraph cluster_superstep_" << superstep << " {" << std::endl;
    file << "    label=\"Superstep " << superstep << "\";" << std::endl;
    file << "    style=filled;" << std::endl;
    file << "    color=lightgrey;" << std::endl;
    file << "    rank=same;"
         << std::endl;  // Force all nodes in this cluster to have the same rank

    // Create subgraphs for each processor within this superstep
    for (unsigned proc = 0; proc < numProcessors; ++proc) {
      file << "    subgraph cluster_superstep_" << superstep << "_proc_" << proc
           << " {" << std::endl;
      file << "      label=\"Processor " << proc << "\";" << std::endl;
      file << "      style=filled;" << std::endl;
      file << "      color=white;" << std::endl;

      // Add nodes for this superstep and processor
      spn.getRoot()->walk([&file, &schedule, superstep, proc](NodeRef node) {
        if (schedule.isScheduled(node) &&
            schedule.getSuperstep(node) == superstep &&
            schedule.getProcessor(node) == proc) {
          file << "      " << getNodeId(node) << " [" << getNodeAttributes(node)
               << "];" << std::endl;
        }
      });

      file << "    }" << std::endl;  // End processor subgraph
    }

    file << "  }" << std::endl;  // End superstep subgraph
  }

  // Define edges
  std::unordered_set<NodeRef> visited;
  spn.getRoot()->walk([&file, &visited](NodeRef node) {
    if (visited.find(node) == visited.end()) {
      writeEdges(file, node);
      visited.insert(node);
    }
  });

  file << "}" << std::endl;
}

}  // namespace spnipu