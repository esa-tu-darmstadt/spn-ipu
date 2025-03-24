add_library(SPNIPUBackend SHARED 
  "LowerToGraphene.cpp"
  "ExecutionEngine.cpp"
)
target_link_libraries(SPNIPUBackend PRIVATE GrapheneTensorDSL GrapheneCodeDSL GrapheneTensorDSL GrapheneUtil popops)
target_link_libraries(SPNIPUBackend PUBLIC SPNIPUModel SPNIPUScheduling)