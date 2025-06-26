#include "libspnipu/util/HtmlVisualization.hpp"

#include <iomanip>
#include <sstream>

#include <nlohmann/json.hpp>

#include "libspnipu/scheduling/BSPScheduleTimings.hpp"
#include "libspnipu/scheduling/PerformanceModel.hpp"

namespace spnipu {

void HtmlVisualization::plotBSPScheduleAsTimeline(BSPSchedule& schedule,
                                                  const std::filesystem::path& filename) {
  PerformanceModel model(schedule.getSPN());
  BSPScheduleTimings timings(schedule, model);
  
  // Create JSON data structure
  nlohmann::json timelineData;
  
  // SPN information
  const SPN& spn = schedule.getSPN();
  auto nodeDistribution = spn.getNodeTypeDistribution();
  timelineData["spn"] = {
    {"name", spn.getName()},
    {"totalNodes", spn.getTotalNodeCount()},
    {"numFeatures", spn.getNumFeatures()},
    {"nodeDistribution", nodeDistribution}
  };
  
  // Schedule information
  timelineData["schedule"] = {
    {"numSupersteps", timings.getNumSupersteps()},
    {"numProcessors", timings.getNumProcessors()},
    {"totalTime", timings.getTotalTime()},
    {"averageUtilization", timings.getAverageUtilization()},
    {"loadImbalance", timings.getLoadImbalance()},
    {"schedulingAlgorithm", schedule.getSchedulingAlgorithmDescription()},
    {"partitioningAlgorithm", schedule.getPartitioningAlgorithmDescription().value_or("None")}
  };
  
  // Add processor timelines
  timelineData["processors"] = nlohmann::json::array();
  for (unsigned proc = 0; proc < timings.getNumProcessors(); ++proc) {
    const auto& timeline = timings.getProcessorTimeline(proc);
    
    nlohmann::json processorData;
    processorData["id"] = proc;
    processorData["utilization"] = timeline.getUtilization();
    processorData["totalComputeTime"] = timeline.totalComputeTime;
    processorData["totalIdleTime"] = timeline.totalIdleTime;
    
    processorData["events"] = nlohmann::json::array();
    for (const auto& event : timeline.events) {
      nlohmann::json eventData;
      eventData["startTime"] = event.startTime;
      eventData["endTime"] = event.endTime;
      eventData["duration"] = event.getDuration();
      eventData["taskType"] = event.taskType;
      eventData["color"] = event.color;
      eventData["isIdle"] = event.isIdle();
      
      // Add node-specific information
      if (!event.isIdle() && event.node) {
        if (auto* gaussianNode = dynamic_cast<GaussianLeafNode*>(event.node)) {
          eventData["nodeInfo"] = {
            {"type", "gaussian"},
            {"scope", gaussianNode->getScope()}
          };
        } else {
          eventData["nodeInfo"] = {
            {"type", event.taskType}
          };
        }
      }
      
      processorData["events"].push_back(eventData);
    }
    
    timelineData["processors"].push_back(processorData);
  }
  
  // Add superstep boundaries
  timelineData["supersteps"] = nlohmann::json::array();
  for (unsigned superstep = 0; superstep < timings.getNumSupersteps(); ++superstep) {
    timelineData["supersteps"].push_back({
      {"id", superstep},
      {"startTime", timings.getSuperstepStartTime(superstep)},
      {"duration", timings.getSuperstepDuration(superstep)},
      {"endTime", timings.getSuperstepEndTime(superstep)}
    });
  }
  
  // Generate HTML file
  std::ofstream file(filename);
  
  writeHtmlDocumentStart(file, "BSP Schedule Timeline");
  writeCommonStyles(file);
  writePlotlyScriptImport(file);
  
  file << "</head>\n<body>\n";
  file << "    <h1>BSP Schedule Timeline</h1>\n";
  
  writeInfoSection(file, "spn-info", "SPN Information");
  writeInfoSection(file, "algo-info", "Algorithms Used"); 
  writeStatsSection(file);
  writePlotContainer(file, "timeline", timings.getNumProcessors() * 50 + 300);
  
  writeJavaScriptStart(file);
  writeJsonData(file, timelineData);
  writeTimelineVisualizationScript(file);
  writeJavaScriptEnd(file);
  
  writeHtmlDocumentEnd(file);
  file.close();
}

void HtmlVisualization::writeHtmlDocumentStart(std::ofstream& file, const std::string& title) {
  file << R"(<!DOCTYPE html>
<html>
<head>
    <title>)" << title << R"(</title>
)";
}

void HtmlVisualization::writeHtmlDocumentEnd(std::ofstream& file) {
  file << R"(</body>
</html>)";
}

void HtmlVisualization::writeCommonStyles(std::ofstream& file) {
  file << R"(    <style>
        body { font-family: Arial, sans-serif; margin: 20px; }
        h1 { color: #333; }
        .info { margin: 10px 0; padding: 10px; background-color: #f5f5f5; border-radius: 5px; }
        .stats { display: flex; gap: 20px; margin: 10px 0; }
        .stat-item { background-color: #e8f4f8; padding: 8px 12px; border-radius: 4px; }
        .node-distribution { margin-left: 20px; font-size: 0.9em; }
    </style>
)";
}

void HtmlVisualization::writePlotlyScriptImport(std::ofstream& file) {
  file << R"(    <script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
)";
}

void HtmlVisualization::writeInfoSection(std::ofstream& file, 
                                         const std::string& sectionId,
                                         const std::string& title) {
  file << R"(    <div class="info">
        <strong>)" << title << R"(:</strong>
        <div id=")" << sectionId << R"("></div>
    </div>
)";
}

void HtmlVisualization::writeStatsSection(std::ofstream& file) {
  file << R"(    <div class="stats">
        <div class="stat-item">Average Utilization: <span id="avg-util"></span></div>
        <div class="stat-item">Load Imbalance: <span id="load-imbalance"></span></div>
    </div>
)";
}

void HtmlVisualization::writePlotContainer(std::ofstream& file, 
                                           const std::string& containerId,
                                           unsigned height) {
  file << R"(    <div id=")" << containerId << R"(" style="width: 100%; height: )" 
       << height << R"(px;"></div>
)";
}

void HtmlVisualization::writeJavaScriptStart(std::ofstream& file) {
  file << R"(    <script>
)";
}

void HtmlVisualization::writeJavaScriptEnd(std::ofstream& file) {
  file << R"(    </script>
)";
}

void HtmlVisualization::writeJsonData(std::ofstream& file, const nlohmann::json& data) {
  file << "        // Embedded visualization data\n";
  file << "        const visualizationData = " << data.dump(2) << ";\n\n";
}

void HtmlVisualization::writeTimelineVisualizationScript(std::ofstream& file) {
  file << R"(        // Update SPN information
        const spnInfo = visualizationData.spn;
        const nodeDistText = Object.entries(spnInfo.nodeDistribution)
            .map(([type, count]) => `${count} ${type}`)
            .join(', ');
        
        document.getElementById('spn-info').innerHTML = `
            <strong>Name:</strong> ${spnInfo.name}<br>
            <strong>Total Nodes:</strong> ${spnInfo.totalNodes} (${nodeDistText})<br>
            <strong>Features:</strong> ${spnInfo.numFeatures}
        `;
        
        // Update algorithm information
        const schedule = visualizationData.schedule;
        document.getElementById('algo-info').innerHTML = `
            <span style="margin-left: 20px;">Partitioning: ${schedule.partitioningAlgorithm}</span><br>
            <span style="margin-left: 20px;">Scheduling: ${schedule.schedulingAlgorithm}</span>
        `;
        
        // Update statistics
        document.getElementById('avg-util').textContent = `${(schedule.averageUtilization * 100).toFixed(1)}%`;
        document.getElementById('load-imbalance').textContent = `${(schedule.loadImbalance * 100).toFixed(1)}%`;
        
        // Create Plotly traces from JSON data
        const traces = [];
        
        // Add processor timelines using horizontal lines (timeline/Gantt style)
        visualizationData.processors.forEach(processor => {
            processor.events.forEach(event => {
                const trace = {
                    x: [event.startTime, event.endTime],
                    y: [processor.id, processor.id],
                    mode: 'lines',
                    line: {
                        color: event.color,
                        width: 20
                    },
                    hovertemplate: `Processor: ${processor.id}<br>` +
                                 `Task: ${event.taskType}<br>` +
                                 `Duration: ${event.duration.toFixed(2)}<br>` +
                                 `Start: ${event.startTime.toFixed(2)}<br>` +
                                 `End: ${event.endTime.toFixed(2)}` +
                                 (event.nodeInfo && event.nodeInfo.scope !== undefined 
                                    ? `<br>Scope: ${event.nodeInfo.scope}` : '') +
                                 '<extra></extra>',
                    name: `${event.taskType}`,
                    showlegend: false
                };
                traces.push(trace);
            });
        });
        
        // Add superstep boundary markers
        traces.push({
            x: visualizationData.supersteps.map(s => s.startTime),
            y: visualizationData.supersteps.map(() => schedule.numProcessors - 1),
            mode: 'markers+text',
            type: 'scatter',
            text: visualizationData.supersteps.map(s => `S${s.id}`),
            textposition: 'top center',
            marker: { color: 'red', size: 8 },
            name: 'Superstep Boundaries',
            showlegend: false,
            hovertemplate: visualizationData.supersteps.map(s => 
                `Superstep ${s.id} start<br>Duration: ${s.duration.toFixed(2)}<extra></extra>`
            )
        });
        
        // Create layout with superstep boundary lines
        const shapes = [];
        for (let i = 1; i < visualizationData.supersteps.length; i++) {
            shapes.push({
                type: 'line',
                x0: visualizationData.supersteps[i].startTime,
                y0: -0.5,
                x1: visualizationData.supersteps[i].startTime,
                y1: schedule.numProcessors - 0.5,
                line: { color: 'red', width: 1, dash: 'dash' }
            });
        }
        
        const layout = {
            title: 'BSP Schedule Timeline - Task Execution and Processor Utilization',
            xaxis: {
                title: 'Time Units',
                type: 'linear',
                showgrid: true,
                gridcolor: '#e1e1e1'
            },
            yaxis: {
                title: 'Processor',
                type: 'linear',
                tickmode: 'array',
                tickvals: Array.from({length: schedule.numProcessors}, (_, i) => i),
                ticktext: Array.from({length: schedule.numProcessors}, (_, i) => `CPU ${i}`),
                range: [-0.5, schedule.numProcessors - 0.5]
            },
            height: )" << "schedule.numProcessors * 50 + 300" << R"(,
            margin: { l: 100, r: 50, t: 80, b: 60 },
            hovermode: 'closest',
            plot_bgcolor: '#fafafa',
            shapes: shapes
        };

        const config = {
            responsive: true,
            displayModeBar: true,
            modeBarButtonsToRemove: ['lasso2d', 'select2d'],
            displaylogo: false
        };

        Plotly.newPlot('timeline', traces, layout, config);
)";
}

}  // namespace spnipu