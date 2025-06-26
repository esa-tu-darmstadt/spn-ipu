add_library(SPNIPUUtil SHARED "DotVisualization.cpp" "HtmlVisualization.cpp")

find_package(nlohmann_json CONFIG REQUIRED)

target_link_libraries(SPNIPUUtil PUBLIC 
  SPNIPUModel
  SPNIPUScheduling
  spdlog::spdlog
  nlohmann_json::nlohmann_json
)