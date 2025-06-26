#pragma once

#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>

#include "libspnipu/scheduling/BSPSchedule.hpp"

namespace spnipu {

/**
 * @brief HTML-based visualization utilities for SPN scheduling data
 *
 * This class provides functions to generate interactive HTML visualizations
 * using modern web technologies like Plotly.js for rich, self-contained
 * visualizations that can be viewed in any web browser.
 */
class HtmlVisualization {
 public:
  /**
   * @brief Generate an interactive HTML timeline visualization of a BSP
   * schedule
   *
   * Creates a self-contained HTML file with an embedded Plotly.js Gantt chart
   * showing:
   * - Task execution timelines per processor
   * - Idle periods and processor utilization
   * - Superstep boundaries and communication patterns
   * - SPN statistics and algorithm information
   * - Interactive hover details and zoom/pan controls
   *
   * @param schedule The BSP schedule to visualize
   * @param filename Output HTML file path
   */
  static void plotBSPScheduleAsTimeline(BSPSchedule& schedule,
                                        const std::filesystem::path& filename);

 private:
  // Generic HTML structure helpers
  static void writeHtmlDocumentStart(std::ofstream& file,
                                     const std::string& title);
  static void writeHtmlDocumentEnd(std::ofstream& file);
  static void writeCommonStyles(std::ofstream& file);
  static void writePlotlyScriptImport(std::ofstream& file);

  // Generic content helpers
  static void writeInfoSection(std::ofstream& file,
                               const std::string& sectionId,
                               const std::string& title);
  static void writeStatsSection(std::ofstream& file);
  static void writePlotContainer(std::ofstream& file,
                                 const std::string& containerId,
                                 unsigned height);

  // JavaScript generation helpers
  static void writeJavaScriptStart(std::ofstream& file);
  static void writeJavaScriptEnd(std::ofstream& file);
  static void writeTimelineVisualizationScript(std::ofstream& file);

  // Data formatting helpers
  static void writeJsonData(std::ofstream& file, const nlohmann::json& data);
};

}  // namespace spnipu