#pragma once

#include <filesystem>
#include <string>
#include <unordered_map>

#include "libspnipu/model/Nodes.hpp"
#include "libspnipu/model/SPN.hpp"
#include "libspnipu/scheduling/BSPSchedule.hpp"

namespace spnipu {

class DotVisualizer {
 public:
  // Plot an SPN to a dot file
  static void plotSPN(SPN& spn, const std::filesystem::path& filename);

  // Plot a BSPSchedule to a dot file with grouped supersteps and processors
  static void plotBSPSchedule(BSPSchedule& schedule,
                              const std::filesystem::path& filename);

 private:
  // Helper for node attributes
  static std::string getNodeAttributes(NodeRef node);

  // Helper to generate node IDs
  static std::string getNodeId(NodeRef node);

  // Helper to generate edge definitions
  static void writeEdges(std::ofstream& file, NodeRef node);
};

}  // namespace spnipu