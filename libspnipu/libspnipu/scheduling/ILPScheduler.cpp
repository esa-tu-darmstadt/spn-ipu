#include "libspnipu/scheduling/ILPScheduler.hpp"

#include <absl/time/time.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <future>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "libspnipu/scheduling/BSPSchedule.hpp"
#include "libspnipu/scheduling/PerformanceModel.hpp"
#include "ortools/linear_solver/linear_solver.h"

using namespace spnipu;
using namespace operations_research;

namespace {
// A helper function to generate variable names.
std::string VarName(const std::string& base, int v, int p, int s) {
  std::ostringstream oss;
  oss << base << "_" << v << "_" << p << "_" << s;
  return oss.str();
}

std::optional<BSPSchedule> scheduleWithILP(SPN& spn, const ILPConfig& config,
                                           const PerformanceModel& model) {
  // Extract all nodes from the SPN
  std::vector<NodeRef> nodes;
  for (const std::unique_ptr<Node>& node : spn.getNodes()) {
    nodes.push_back(node.get());
  }
  std::unordered_map<NodeRef, int> nodeToIndex;
  for (int i = 0; i < nodes.size(); i++) {
    nodeToIndex[nodes[i]] = i;
  }

  // Create the MPSolver instance.
  std::unique_ptr<MPSolver> solver;
  switch (config.solver) {
    case ILPSolver::Gurobi:
      solver = std::make_unique<MPSolver>(
          "SPN-IPU-Scheduler", MPSolver::GUROBI_MIXED_INTEGER_PROGRAMMING);
      break;
    case ILPSolver::CBC:
      solver = std::make_unique<MPSolver>(
          "SPN-IPU-Scheduler", MPSolver::CBC_MIXED_INTEGER_PROGRAMMING);
      break;
    case ILPSolver::GLPK:
      solver = std::make_unique<MPSolver>(
          "SPN-IPU-Scheduler", MPSolver::GLPK_MIXED_INTEGER_PROGRAMMING);
      break;
    case ILPSolver::HIGHS:
      solver = std::make_unique<MPSolver>(
          "SPN-IPU-Scheduler", MPSolver::HIGHS_MIXED_INTEGER_PROGRAMMING);
      break;
    default:
      throw std::invalid_argument("Unsupported solver type.");
  }

  if (!solver) {
    throw std::runtime_error("Failed to create MPSolver instance.");
  }

  spdlog::debug("Scheduling SPN using {} solver",
                ToString(solver->ProblemType()));

  const int n = nodes.size();
  const int P = config.maxProcessors;
  const int S = config.maxSupersteps;

  // 3D arrays (node x processor x superstep) for computation, presence, sent,
  // and recv.
  std::vector<std::vector<std::vector<const MPVariable*>>> comp(
      n, std::vector<std::vector<const MPVariable*>>(
             P, std::vector<const MPVariable*>(S, nullptr)));
  std::vector<std::vector<std::vector<const MPVariable*>>> pres(
      n, std::vector<std::vector<const MPVariable*>>(
             P, std::vector<const MPVariable*>(S, nullptr)));
  std::vector<std::vector<std::vector<const MPVariable*>>> sent(
      n, std::vector<std::vector<const MPVariable*>>(
             P, std::vector<const MPVariable*>(S, nullptr)));
  std::vector<std::vector<std::vector<const MPVariable*>>> recv(
      n, std::vector<std::vector<const MPVariable*>>(
             P, std::vector<const MPVariable*>(S, nullptr)));

  // Create binary variables for each node, processor, and superstep.
  for (int v = 0; v < n; v++) {
    for (int p = 0; p < P; p++) {
      for (int s = 0; s < S; s++) {
        comp[v][p][s] = solver->MakeBoolVar(VarName("comp", v, p, s));
        pres[v][p][s] = solver->MakeBoolVar(VarName("pres", v, p, s));
        sent[v][p][s] = solver->MakeBoolVar(VarName("sent", v, p, s));
        recv[v][p][s] = solver->MakeBoolVar(VarName("recv", v, p, s));
      }
    }
  }

  // *** Add Constraints ***

  // (1) Each node is computed exactly once.
  for (int v = 0; v < n; v++) {
    LinearExpr sumComp;
    for (int p = 0; p < P; p++) {
      for (int s = 0; s < S; s++) {
        sumComp += comp[v][p][s];
      }
    }
    solver->MakeRowConstraint(sumComp == 1);
  }

  // (2) Presence propagation: for s = 0, pres = comp; for s>=1, pres[v][p][s]
  // <= pres[v][p][s-1] + comp[v][p][s] + recv[v][p][s-1].
  for (int v = 0; v < n; v++) {
    for (int p = 0; p < P; p++) {
      // For s = 0:
      solver->MakeRowConstraint(LinearExpr(pres[v][p][0]) ==
                                LinearExpr(comp[v][p][0]));
      // For s >= 1:
      for (int s = 1; s < S; s++) {
        LinearExpr rhs = LinearExpr(pres[v][p][s - 1]) +
                         LinearExpr(comp[v][p][s]) +
                         LinearExpr(recv[v][p][s - 1]);
        solver->MakeRowConstraint(pres[v][p][s] <= rhs);
      }
    }
  }

  // (3) Precedence: for every edge (u,v) and for each p, s, require that if v
  // is computed on p in superstep s then u’s value is present on p.
  for (NodeRef v : nodes) {
    for (NodeRef u : v->getChildren()) {
      // Find indices for u and v in nodeList. (Assume we have a mapping;
      // omitted here for brevity.)
      int uIndex = nodeToIndex[u];
      int vIndex = nodeToIndex[v];
      for (int p = 0; p < P; p++) {
        for (int s = 0; s < S; s++) {
          solver->MakeRowConstraint(LinearExpr(comp[vIndex][p][s]) <=
                                    LinearExpr(pres[uIndex][p][s]));
        }
      }
    }
  }

  // (4) Communication constraints:
  // A processor can only send a node if it has it.
  for (int v = 0; v < n; v++) {
    for (int p = 0; p < P; p++) {
      for (int s = 0; s < S; s++) {
        solver->MakeRowConstraint(LinearExpr(sent[v][p][s]) <=
                                  LinearExpr(pres[v][p][s]));
      }
    }
  }
  // And if a processor receives a value, then some other processor must
  // have sent it.
  for (int v = 0; v < n; v++) {
    for (int p = 0; p < P; p++) {
      for (int s = 0; s < S; s++) {
        LinearExpr otherSends;
        for (int p2 = 0; p2 < P; p2++) {
          if (p2 != p) otherSends += sent[v][p2][s];
        }
        solver->MakeRowConstraint(LinearExpr(recv[v][p][s]) <= otherSends);
      }
    }
  }

  // (5) Define work cost per superstep.
  std::vector<const MPVariable*> Work(S, nullptr);
  for (int s = 0; s < S; s++) {
    Work[s] =
        solver->MakeNumVar(0, solver->infinity(), "Work_" + std::to_string(s));
    for (int p = 0; p < P; p++) {
      LinearExpr work_p;
      for (int v = 0; v < n; v++) {
        NodeRef node = nodes[v];
        int w = model.getComputationCost(node, p);
        work_p += LinearExpr(comp[v][p][s]) * w;
      }
      // Work[s] >= work_p for all processors.
      solver->MakeRowConstraint(Work[s] >= work_p);
    }
  }

  // (6) Define communication cost per superstep.
  std::vector<const MPVariable*> Comm(S, nullptr);
  for (int s = 0; s < S; s++) {
    Comm[s] =
        solver->MakeNumVar(0, solver->infinity(), "Comm_" + std::to_string(s));
    for (int p = 0; p < P; p++) {
      LinearExpr sent_p;
      LinearExpr recv_p;
      for (int v = 0; v < n; v++) {
        sent_p += sent[v][p][s];
        recv_p += recv[v][p][s];
      }
      solver->MakeRowConstraint(Comm[s] >= sent_p);
      solver->MakeRowConstraint(Comm[s] >= recv_p);
    }
  }

  // (7) Set the objective: minimize total cost over supersteps.
  LinearExpr totalCost;
  for (int s = 0; s < S; s++) {
    int g = model.getCommunicationCost();
    int L = 0;  // TODO: Latency of empty supersteps must be considered.
    totalCost += LinearExpr(Work[s]) + g * LinearExpr(Comm[s]) + L;
  }
  MPObjective* const objective = solver->MutableObjective();
  objective->OptimizeLinearExpr(totalCost, false);

  // Export model to LP format (optional)
  // std::string model_str;
  // if (solver->ExportModelAsLpFormat(false, &model_str)) {
  //   spdlog::info("LP Model format:");
  //   spdlog::info("{}", model_str);
  // }

  // Setup solver parameters
  if (config.enableOutput) solver->EnableOutput();
  if (config.threads > 0)
    solver->SetSolverSpecificParametersAsString("Threads " +
                                                std::to_string(config.threads));
  if (config.timeLimitSeconds > 0)
    solver->SetTimeLimit(absl::Seconds(config.timeLimitSeconds));

  // Solve the ILP problem
  spdlog::trace("ILP problem constructed with {} variables and {} constraints",
                solver->NumVariables(), solver->NumConstraints());
  spdlog::info("Starting to solve the ILP Problem. Press enter to interrupt.");

  // Launch the solver in an asynchronous task.
  auto solveFuture =
      std::async(std::launch::async, [&]() { return solver->Solve(); });

  // Launch another asynchronous task to wait for the enter key press.
  auto inputFuture = std::async(std::launch::async, []() {
    std::cin.get();  // Wait for user to press enter.
    return true;
  });

  // Poll periodically to check if the user pressed enter.
  MPSolver::ResultStatus result_status;
  bool interrupted = false;
  while (solveFuture.wait_for(std::chrono::milliseconds(100)) !=
         std::future_status::ready) {
    if (inputFuture.wait_for(std::chrono::milliseconds(0)) ==
        std::future_status::ready) {
      spdlog::info("User pressed enter. Interrupting solver.");
      solver->InterruptSolve();
      interrupted = true;
      break;
    }
  }
  // Wait for the solver to finish.
  result_status = solveFuture.get();

  if (interrupted) {
    spdlog::info("Solver was interrupted by the user.");
  }

  if (result_status == MPSolver::OPTIMAL ||
      result_status == MPSolver::FEASIBLE) {
    spdlog::info("Solution ({}) found after {} s with objective value: {}",
                 ProtoEnumToString<MPSolverResponseStatus>(
                     static_cast<MPSolverResponseStatus>(result_status)),
                 absl::ToDoubleSeconds(solver->DurationSinceConstruction()),
                 objective->Value());

    // Extract the solution and create a BSP schedule
    BSPSchedule schedule(spn);
    for (NodeRef node : nodes) {
      int v = nodeToIndex[node];
      for (unsigned p = 0; p < P; p++) {
        for (unsigned s = 0; s < S; s++) {
          if (comp[v][p][s]->solution_value() > 0.5) {
            schedule.scheduleNode(node, s, p);
          }
        }
      }
    }

    schedule.lock();
    schedule.validate();
    spdlog::trace("Schedule validated");
    return schedule.withoutEmptySupersteps();
  } else {
    spdlog::error("No solution found after {} s: {}",
                  absl::ToDoubleSeconds(solver->DurationSinceConstruction()),
                  ProtoEnumToString<MPSolverResponseStatus>(
                      static_cast<MPSolverResponseStatus>(result_status)));
    return std::nullopt;
  }
}

}  // namespace

std::optional<BSPSchedule> spnipu::scheduleWithILP(SPN& spn,
                                                   const ILPConfig& config) {
  PerformanceModel model(spn);
  return ::scheduleWithILP(spn, config, model);
}