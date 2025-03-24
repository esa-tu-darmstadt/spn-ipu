add_library(SPNIPUFrontend Deserialize.cpp)
target_link_libraries(SPNIPUFrontend PUBLIC SPNIPUModel)

# Fetch capnproto and generate files
FetchContent_Declare(
  capnproto
  GIT_REPOSITORY    git@github.com:capnproto/capnproto.git
  GIT_TAG           v1.1.0
)
FetchContent_MakeAvailable(capnproto)

capnp_generate_cpp(CAPNP_SRCS CAPNP_HDRS spflow.capnp)
message(STATUS "Capnproto headers: ${CAPNP_HDRS}")
message(STATUS "Capnproto sources: ${CAPNP_SRCS}")

target_link_libraries(SPNIPUFrontend PRIVATE CapnProto::capnp)
target_sources(SPNIPUFrontend PRIVATE ${CAPNP_SRCS})
target_include_directories(SPNIPUFrontend PRIVATE ${CMAKE_CURRENT_BINARY_DIR})