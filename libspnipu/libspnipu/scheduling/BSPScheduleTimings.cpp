#include "libspnipu/scheduling/BSPScheduleTimings.hpp"

#include <algorithm>
#include <functional>
#include <numeric>
#include <sstream>
#include <queue>
#include <unordered_map>

namespace spnipu {

BSPScheduleTimings::BSPScheduleTimings(const BSPSchedule& schedule,
                                       PerformanceModel& model)
    : schedule_(schedule), model_(model) {
  numSupersteps_ = schedule_.getNumSupersteps();
  numProcessors_ = schedule_.getNumProcessors();

  calculateSuperstepTimings();
  calculateProcessorTimelines();
}

double BSPScheduleTimings::getSuperstepStartTime(unsigned superstep) const {
  if (superstep >= superstepStartTimes_.size()) return 0.0;
  return superstepStartTimes_[superstep];
}

double BSPScheduleTimings::getSuperstepDuration(unsigned superstep) const {
  if (superstep >= superstepDurations_.size()) return 0.0;
  return superstepDurations_[superstep];
}

double BSPScheduleTimings::getSuperstepEndTime(unsigned superstep) const {
  return getSuperstepStartTime(superstep) + getSuperstepDuration(superstep);
}

const BSPScheduleTimings::ProcessorTimeline&
BSPScheduleTimings::getProcessorTimeline(unsigned processor) const {
  static ProcessorTimeline empty{processor, {}, 0.0, 0.0};
  if (processor >= processorTimelines_.size()) return empty;
  return processorTimelines_[processor];
}

double BSPScheduleTimings::getProcessorUtilization(unsigned processor) const {
  return getProcessorTimeline(processor).getUtilization();
}

std::vector<BSPScheduleTimings::TimelineEvent>
BSPScheduleTimings::getAllEvents() const {
  std::vector<TimelineEvent> allEvents;
  for (const auto& timeline : processorTimelines_) {
    allEvents.insert(allEvents.end(), timeline.events.begin(),
                     timeline.events.end());
  }
  return allEvents;
}

double BSPScheduleTimings::getAverageUtilization() const {
  if (processorTimelines_.empty()) return 0.0;

  double totalUtilization = 0.0;
  for (const auto& timeline : processorTimelines_) {
    totalUtilization += timeline.getUtilization();
  }
  return totalUtilization / processorTimelines_.size();
}

double BSPScheduleTimings::getLoadImbalance() const {
  if (processorTimelines_.empty()) return 0.0;

  std::vector<double> utilizations;
  for (const auto& timeline : processorTimelines_) {
    utilizations.push_back(timeline.getUtilization());
  }

  auto minMax = std::minmax_element(utilizations.begin(), utilizations.end());
  double maxUtil = *minMax.second;
  double minUtil = *minMax.first;

  return maxUtil > 0.0 ? (maxUtil - minUtil) / maxUtil : 0.0;
}

void BSPScheduleTimings::calculateSuperstepTimings() {
  superstepStartTimes_.resize(numSupersteps_, 0.0);
  superstepDurations_.resize(numSupersteps_, 0.0);

  // Calculate superstep durations (BSP model: duration = max processor time)
  for (unsigned superstep = 0; superstep < numSupersteps_; ++superstep) {
    double maxProcTime = 0.0;

    const auto& superstepNodes = schedule_.getNodesOfSuperstep(superstep);
    for (const auto& [proc, nodes] : superstepNodes) {
      double procTime = 0.0;
      for (NodeRef node : nodes) {
        procTime += model_.getComputationCost(node, proc);
      }
      maxProcTime = std::max(maxProcTime, procTime);
    }

    superstepDurations_[superstep] =
        std::max(maxProcTime, 1.0);  // Minimum 1 time unit
  }

  // Calculate start times (sequential supersteps)
  for (unsigned superstep = 1; superstep < numSupersteps_; ++superstep) {
    superstepStartTimes_[superstep] = superstepStartTimes_[superstep - 1] +
                                      superstepDurations_[superstep - 1];
  }

  totalTime_ = superstepStartTimes_.back() + superstepDurations_.back();
}

std::vector<NodeRef> BSPScheduleTimings::topologicalSort(
    const std::unordered_set<NodeRef>& nodes) const {
  std::vector<NodeRef> result;
  std::unordered_set<NodeRef> visited;
  std::unordered_set<NodeRef> inStack;

  // Helper function for DFS-based topological sort
  std::function<void(NodeRef)> dfs = [&](NodeRef node) {
    if (inStack.count(node)) {
      // Cycle detected - this shouldn't happen in a valid SPN, but handle gracefully
      return;
    }
    if (visited.count(node)) {
      return;
    }

    visited.insert(node);
    inStack.insert(node);

    // Visit children first (children should be processed before parents)
    for (NodeRef child : node->getChildren()) {
      if (nodes.count(child)) {  // Only consider children that are in our subset
        dfs(child);
      }
    }

    inStack.erase(node);
    result.push_back(node);  // Add to result after visiting all children
  };

  // Process all nodes in the subset
  for (NodeRef node : nodes) {
    if (!visited.count(node)) {
      dfs(node);
    }
  }

  return result;
}

void BSPScheduleTimings::calculateProcessorTimelines() {
  processorTimelines_.resize(numProcessors_);

  for (unsigned proc = 0; proc < numProcessors_; ++proc) {
    ProcessorTimeline& timeline = processorTimelines_[proc];
    timeline.processor = proc;
    timeline.totalComputeTime = 0.0;
    timeline.totalIdleTime = 0.0;

    for (unsigned superstep = 0; superstep < numSupersteps_; ++superstep) {
      double superstepStart = superstepStartTimes_[superstep];
      double currentTime = superstepStart;

      const auto& superstepNodes = schedule_.getNodesOfSuperstep(superstep);
      auto procIt = superstepNodes.find(proc);

      if (procIt != superstepNodes.end() && !procIt->second.empty()) {
        // Processor has tasks in this superstep - sort them topologically
        std::vector<NodeRef> sortedNodes = topologicalSort(procIt->second);
        
        for (NodeRef node : sortedNodes) {
          double taskDuration = model_.getComputationCost(node, proc);

          TimelineEvent event;
          event.processor = proc;
          event.startTime = currentTime;
          event.endTime = currentTime + taskDuration;
          event.node = node;
          event.taskType = getNodeType(node);
          event.color = getNodeColor(node);

          timeline.events.push_back(event);
          timeline.totalComputeTime += taskDuration;
          currentTime += taskDuration;
        }

        // Add idle time at end of superstep if needed
        double superstepEnd = superstepStart + superstepDurations_[superstep];
        if (currentTime < superstepEnd) {
          double idleDuration = superstepEnd - currentTime;

          TimelineEvent idleEvent;
          idleEvent.processor = proc;
          idleEvent.startTime = currentTime;
          idleEvent.endTime = superstepEnd;
          idleEvent.node = nullptr;
          idleEvent.taskType = "idle";
          idleEvent.color = "#ecf0f1";

          timeline.events.push_back(idleEvent);
          timeline.totalIdleTime += idleDuration;
        }
      } else {
        // Processor is idle for entire superstep
        double superstepEnd = superstepStart + superstepDurations_[superstep];
        double idleDuration = superstepDurations_[superstep];

        TimelineEvent idleEvent;
        idleEvent.processor = proc;
        idleEvent.startTime = superstepStart;
        idleEvent.endTime = superstepEnd;
        idleEvent.node = nullptr;
        idleEvent.taskType = "idle";
        idleEvent.color = "#ecf0f1";

        timeline.events.push_back(idleEvent);
        timeline.totalIdleTime += idleDuration;
      }
    }
  }
}

std::string BSPScheduleTimings::getNodeColor(NodeRef node) const {
  if (!node) return "#ecf0f1";  // Light gray for idle

  if (dynamic_cast<SumNode*>(node)) {
    return "#3498db";  // Blue
  } else if (dynamic_cast<ProductNode*>(node)) {
    return "#2ecc71";  // Green
  } else if (dynamic_cast<GaussianLeafNode*>(node)) {
    return "#f39c12";  // Orange
  } else {
    return "#95a5a6";  // Gray for unknown
  }
}

std::string BSPScheduleTimings::getNodeType(NodeRef node) const {
  if (!node) return "idle";

  if (dynamic_cast<SumNode*>(node)) {
    return "sum";
  } else if (dynamic_cast<ProductNode*>(node)) {
    return "product";
  } else if (dynamic_cast<GaussianLeafNode*>(node)) {
    return "gaussian";
  } else {
    return "unknown";
  }
}

}  // namespace spnipu