#pragma once

#include <memory>
#include <span>

#include "libspnipu/scheduling/BSPSchedule.hpp"

namespace graphene {
class Runtime;
}

namespace poplar {
class Engine;
}

namespace spnipu {
class ExecutionEngine {
  const BSPSchedule &schedule_;
  const SPN &spn_;

  std::unique_ptr<graphene::Runtime> runtime_;
  std::unique_ptr<poplar::Engine> engine_;

 public:
  ExecutionEngine(const BSPSchedule &schedule);
  ~ExecutionEngine();

  void compile(bool verbose = false);

  /// Run the SPN on the input data, i.e., calculates the joint probability of
  /// the SPN with the given features. Does not yet support batching.
  float run(std::span<float> features);
};
}  // namespace spnipu