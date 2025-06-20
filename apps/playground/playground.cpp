#include <iostream>

#include "libspnipu/backend/ExecutionEngine.hpp"
#include "libspnipu/frontend/Deserialize.hpp"
#include "libspnipu/model/Nodes.hpp"
#include "libspnipu/model/SPN.hpp"
#include "libspnipu/scheduling/BSPSchedule.hpp"
#include "libspnipu/scheduling/ILPScheduler.hpp"
#include "libspnipu/util/HostExecutor.hpp"
#include "libspnipu/util/Visualization.hpp"
#include "spdlog/spdlog.h"

using namespace spnipu;
int main(int argc, char** argv) {
  spdlog::set_level(spdlog::level::trace);

  // SPN spn = deserializeSPN("/workspaces/spn-ipu/data/spns/gaussian10.bin");
  SPN spn = deserializeSPN("/workspaces/spn-ipu/data/spns/gaussian10.bin");

  DotVisualizer::plotSPN(spn, "spn.dot");

  std::vector<double> hostInput(spn.getNumFeatures(), 0.5);
  std::vector<float> deviceInput(spn.getNumFeatures());
  std::copy(hostInput.begin(), hostInput.end(), deviceInput.begin());

  HostExecutor hostExecutor(spn);
  double hostResult = hostExecutor.evaluate(hostInput);

  // BSPSchedule schedule(spn);
  // {
  //   // Schedule the nodes in post-order
  //   unsigned superstep = 0;
  //   spn.walk<NodeTraversalOrder::PostOrder>(
  //       [&schedule, &superstep](NodeRef node) {
  //         //   if (dynamic_cast<LeafNode*>(node) != 0)
  //         //     schedule.scheduleNode(node, 0, 0);
  //         //   else
  //         //     schedule.scheduleNode(node, 1, 0);
  //         schedule.scheduleNode(node, (superstep++) / 3, 0);
  //       });
  // }

  std::optional<BSPSchedule> maybeSchedule = scheduleWithILP(spn);
  if (!maybeSchedule.has_value()) {
    spdlog::error("Failed to schedule SPN");
    return 1;
  }
  BSPSchedule& schedule = maybeSchedule.value();
  DotVisualizer::plotBSPSchedule(schedule, "schedule.dot");

  schedule.lock();
  schedule.validate();

  ExecutionEngine engine(schedule);
  engine.compile(true);
  float deviceResult = engine.run(deviceInput);

  spdlog::info("Host result: {}", hostResult);
  spdlog::info("Device result: {}", deviceResult);

  return 0;
}