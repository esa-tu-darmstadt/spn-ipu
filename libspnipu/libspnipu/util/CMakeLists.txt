add_library(SPNIPUUtil SHARED "Visualization.cpp")

target_link_libraries(SPNIPUUtil PUBLIC 
  SPNIPUModel
  SPNIPUScheduling
  spdlog::spdlog
)