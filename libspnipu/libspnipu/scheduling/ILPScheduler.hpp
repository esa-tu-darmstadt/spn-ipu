#pragma once

#include "libspnipu/scheduling/BSPSchedule.hpp"

namespace spnipu {
enum class ILPSolver { Gurobi, CBC, GLPK, HIGHS };
struct ILPConfig {
  ILPSolver solver = ILPSolver::Gurobi;
  unsigned maxProcessors = 10;
  unsigned maxSupersteps = 5;
  int timeLimitSeconds = -1;
  int threads = 128;
  bool enableOutput = true;
};

std::optional<BSPSchedule> scheduleWithILP(SPN &spn,
                                           const ILPConfig &config = {});
}  // namespace spnipu