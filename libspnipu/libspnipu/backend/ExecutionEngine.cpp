#include "libspnipu/backend/ExecutionEngine.hpp"

#include <spdlog/spdlog.h>

#include <poplar/StreamCallback.hpp>

#include "libgraphene/util/Runtime.hpp"
#include "libspnipu/backend/LowerToGraphene.hpp"

using namespace spnipu;
using namespace graphene;

ExecutionEngine::ExecutionEngine(const BSPSchedule &schedule)
    : schedule_(schedule), spn_(schedule.getSPN()) {}

ExecutionEngine::~ExecutionEngine() {}

void ExecutionEngine::compile(bool verbose) {
  // initialize the runtime
  size_t numIPUs = 1;
  runtime_ = std::make_unique<Runtime>(numIPUs);

  lowerToGraphene(schedule_, verbose);

  // compile the graph
  engine_ = std::make_unique<poplar::Engine>(runtime_->compileGraph());
}

float ExecutionEngine::run(std::span<float> features) {
  if (!engine_) {
    throw std::runtime_error("Execution engine not compiled");
  }
  struct InputStreamCallback : public poplar::StreamCallback {
    std::span<float> features;
    InputStreamCallback(std::span<float> features) : features(features) {}
    Result prefetch(void *ptr) final {
      spdlog::trace("Input stream prefetching called");
      memcpy(ptr, features.data(), features.size() * sizeof(float));
      return Result::Success;
    }
    void fetch(void *ptr) final {
      spdlog::trace("Input stream fetching called");
      memcpy(ptr, features.data(), features.size() * sizeof(float));
    }
    void complete() final { spdlog::trace("Input stream completion called"); }
  };
  struct OutputStreamCallback : public poplar::StreamCallback {
    float &result;
    OutputStreamCallback(float &result) : result(result) {}
    Result prefetch(void *p) final {
      spdlog::error("Output stream prefetching called. This should not happen");
      return Result::NotAvailable;
    }
    void fetch(void *ptr) final {
      spdlog::trace("Output stream fetching called");
      result = *(float *)ptr;
    }
    void complete() final { spdlog::trace("Output stream completion called"); }
  };

  float result = 0.0f;
  auto inputStreamCallback = std::make_unique<InputStreamCallback>(features);
  auto outputStreamCallback = std::make_unique<OutputStreamCallback>(result);

  engine_->connectStreamToCallback("inputStream",
                                   std::move(inputStreamCallback));
  engine_->connectStreamToCallback("outputStream",
                                   std::move(outputStreamCallback));

  runtime_->loadAndRunEngine(*engine_);

  return result;
}