#include <iostream>

#include "libspnipu/backend/ExecutionEngine.hpp"
#include "libspnipu/frontend/Deserialize.hpp"
#include "libspnipu/model/Nodes.hpp"
#include "libspnipu/model/Partitioning.hpp"
#include "libspnipu/model/SPN.hpp"
#include "libspnipu/scheduling/BSPSchedule.hpp"
#include "libspnipu/scheduling/ILPScheduler.hpp"
#include "libspnipu/scheduling/Partitioning.hpp"
#include "libspnipu/util/DotVisualization.hpp"
#include "libspnipu/util/HostExecutor.hpp"
#include "libspnipu/util/HtmlVisualization.hpp"
#include "spdlog/spdlog.h"

using namespace spnipu;
int main(int argc, char** argv) {
  spdlog::set_level(spdlog::level::trace);

  //   SPN spn = deserializeSPN(
  //       "/workspaces/spn-ipu/data/spns/plants_100_200_4_3_3_3_1_True.bin");
  // SPN spn = deserializeSPN("/workspaces/spn-ipu/data/spns/gaussian10.bin");
  SPN spn = deserializeSPN("/workspaces/spn-ipu/data/spns/speaker_FADG0.bin");

  plotSPNAsDot(spn, "spn.dot");

  std::vector<double> hostInput(spn.getNumFeatures(), 0.5);
  std::vector<float> deviceInput(spn.getNumFeatures());
  std::copy(hostInput.begin(), hostInput.end(), deviceInput.begin());

  HostExecutor hostExecutor(spn, false);
  double hostResult = hostExecutor.evaluate(hostInput);

  // // Calculate the schedule without partitioning
  // std::optional<BSPSchedule> scheduleWithoutPartitioning =
  // scheduleWithILP(spn);
  // plotBSPScheduleAsDot(*scheduleWithoutPartitioning,
  //                      "scheduleWithoutPartitioning.dot");

  // Partition the SPN using DagP
  std::optional<Partitioning> partitioning =
      partitionWithDagP(spn, std::min(spn.getNodes().size(), 50ul));
  plotPartitioningAsDot(*partitioning, "partitioning_detailed.dot", true);
  plotPartitioningAsDot(*partitioning, "partitioning_simplified.dot", false);

  // Calculate the schedule with partitioning
  std::optional<BSPSchedule> scheduleWithPartitioning =
      scheduleWithILP(*partitioning);
  plotBSPScheduleAsDot(*scheduleWithPartitioning,
                       "scheduleWithPartitioning_simplified.dot", false);
  plotBSPScheduleAsDot(*scheduleWithPartitioning,
                       "scheduleWithPartitioning_detailed.dot", true);
  HtmlVisualization::plotBSPScheduleAsTimeline(
      *scheduleWithPartitioning, "scheduleWithPartitioning_timeline.html");

  ExecutionEngine engine(*scheduleWithPartitioning);
  engine.compile(true);
  float deviceResult = engine.run(deviceInput);

  spdlog::info("Host result: {}", hostResult);
  spdlog::info("Device result: {}", deviceResult);

  return 0;
}