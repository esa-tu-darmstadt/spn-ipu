add_library(SPNIPUScheduling SHARED "BSPSchedule.cpp")

  target_link_libraries(SPNIPUScheduling PUBLIC Boost::boost)