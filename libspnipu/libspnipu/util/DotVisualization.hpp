#pragma once

#include <filesystem>

#include "libspnipu/model/Partitioning.hpp"
#include "libspnipu/model/SPN.hpp"
#include "libspnipu/scheduling/BSPSchedule.hpp"

namespace spnipu {

// Plot an SPN to a dot file
void plotSPNAsDot(SPN& spn, const std::filesystem::path& filename);

// Plot a BSPSchedule to a dot file with grouped supersteps and processors
void plotBSPScheduleAsDot(BSPSchedule& schedule,
                          const std::filesystem::path& filename,
                          bool showIndividualNodes = true);

// Plot a Partitioning to a dot file with partitions as subgraphs
void plotPartitioningAsDot(Partitioning& partitioning,
                           const std::filesystem::path& filename,
                           bool showIndividualNodes = true);

// Note: HTML timeline visualization moved to HtmlVisualization class

}  // namespace spnipu