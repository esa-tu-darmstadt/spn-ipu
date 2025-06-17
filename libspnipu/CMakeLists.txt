## -------- Graphene --------
option(USE_GRAPHENE_SUBMODULE "Use graphene as a submodule" ON)
if (USE_GRAPHENE_SUBMODULE)
  message(STATUS "Using graphene as a submodule")
  add_subdirectory(libs/graphene EXCLUDE_FROM_ALL)
else()
  message(STATUS "Using graphene as a package")
  find_package(graphene REQUIRED)
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(libspnipu/model)
add_subdirectory(libspnipu/frontend)
add_subdirectory(libspnipu/scheduling)
add_subdirectory(libspnipu/backend)
add_subdirectory(libspnipu/util)

add_library(SPNIPU INTERFACE)
target_link_libraries(SPNIPU INTERFACE SPNIPUModel SPNIPUBackend SPNIPUScheduling SPNIPUFrontend SPNIPUUtil)
target_include_directories(SPNIPU INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
