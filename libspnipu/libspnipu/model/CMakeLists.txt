add_library(SPNIPUModel SHARED 
  "Nodes.cpp"
)

target_link_libraries(SPNIPUModel 
  PUBLIC 
    Boost::boost
)

target_link_libraries(SPNIPUModel 
  PRIVATE 
    spdlog::spdlog
)