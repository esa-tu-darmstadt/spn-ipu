add_executable(spnipu-playground 
  "playground.cpp"
)

target_link_libraries(spnipu-playground SPNIPU GrapheneTensorDSL GrapheneUtil spdlog::spdlog)