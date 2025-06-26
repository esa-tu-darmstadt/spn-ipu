#pragma once

#include <string>
#include <vector>

#include "libspnipu/model/Nodes.hpp"
#include "libspnipu/scheduling/BSPSchedule.hpp"
#include "libspnipu/scheduling/PerformanceModel.hpp"

namespace spnipu {

/**
 * @brief Calculates and provides timing information for BSP schedules
 * 
 * This class analyzes a BSP schedule and performance model to compute:
 * - Superstep start times and durations
 * - Processor utilization timelines
 * - Task execution windows
 * - Idle periods
 * - Load balancing statistics
 */
class BSPScheduleTimings {
public:
  /**
   * @brief Represents a single timing event (task execution or idle period)
   */
  struct TimelineEvent {
    unsigned processor;     ///< Processor ID this event occurs on
    double startTime;       ///< Start time of the event
    double endTime;         ///< End time of the event
    NodeRef node;          ///< SPN node being executed (nullptr for idle)
    std::string taskType;   ///< Human-readable task type ("sum", "product", "gaussian", "idle")
    std::string color;      ///< Hex color code for visualization
    
    /// @brief Duration of this event
    double getDuration() const { return endTime - startTime; }
    
    /// @brief True if this is an idle period
    bool isIdle() const { return node == nullptr; }
  };

  /**
   * @brief Timeline of events for a single processor
   */
  struct ProcessorTimeline {
    unsigned processor;                  ///< Processor ID
    std::vector<TimelineEvent> events;   ///< Ordered list of events
    double totalComputeTime;             ///< Total time spent computing
    double totalIdleTime;                ///< Total idle time
    
    /// @brief Processor utilization (0.0 to 1.0)
    double getUtilization() const { 
      double total = totalComputeTime + totalIdleTime;
      return total > 0.0 ? totalComputeTime / total : 0.0;
    }
  };

public:
  /**
   * @brief Construct timing analysis from BSP schedule and performance model
   * @param schedule The BSP schedule to analyze
   * @param model Performance model for timing estimates
   */
  BSPScheduleTimings(const BSPSchedule& schedule, PerformanceModel& model);

  // === Superstep Timing Queries ===
  
  /// @brief Total execution time across all supersteps
  double getTotalTime() const { return totalTime_; }
  
  /// @brief Start time of a specific superstep
  double getSuperstepStartTime(unsigned superstep) const;
  
  /// @brief Duration of a specific superstep
  double getSuperstepDuration(unsigned superstep) const;
  
  /// @brief End time of a specific superstep
  double getSuperstepEndTime(unsigned superstep) const;

  // === Processor Timeline Queries ===
  
  /// @brief Get complete timeline for a specific processor
  const ProcessorTimeline& getProcessorTimeline(unsigned processor) const;
  
  /// @brief Get utilization ratio for a specific processor (0.0 to 1.0)
  double getProcessorUtilization(unsigned processor) const;
  
  /// @brief Get all timeline events across all processors
  std::vector<TimelineEvent> getAllEvents() const;

  // === Statistics ===
  
  /// @brief Average utilization across all processors
  double getAverageUtilization() const;
  
  /// @brief Load imbalance metric (higher = more imbalanced)
  double getLoadImbalance() const;
  
  /// @brief Number of supersteps in the schedule
  unsigned getNumSupersteps() const { return numSupersteps_; }
  
  /// @brief Number of processors in the schedule
  unsigned getNumProcessors() const { return numProcessors_; }

private:
  // References to input data
  const BSPSchedule& schedule_;
  PerformanceModel& model_;
  
  // Computed timing data
  unsigned numSupersteps_;
  unsigned numProcessors_;
  double totalTime_;
  
  std::vector<double> superstepStartTimes_;
  std::vector<double> superstepDurations_;
  std::vector<ProcessorTimeline> processorTimelines_;

  // Internal computation methods
  void calculateSuperstepTimings();
  void calculateProcessorTimelines();
  
  // Utility methods for node ordering
  std::vector<NodeRef> topologicalSort(const std::unordered_set<NodeRef>& nodes) const;
  
  // Utility methods for visualization
  std::string getNodeColor(NodeRef node) const;
  std::string getNodeType(NodeRef node) const;
};

}  // namespace spnipu