find_package(ortools CONFIG REQUIRED)

add_library(SPNIPUScheduling SHARED "BSPSchedule.cpp" "ILPScheduler.cpp")

target_link_libraries(SPNIPUScheduling PUBLIC Boost::boost)
target_link_libraries(SPNIPUScheduling PRIVATE ortools::ortools)