#include "libspnipu/scheduling/ILPScheduler.hpp"

#include <absl/time/time.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "libspnipu/scheduling/BSPSchedule.hpp"
#include "ortools/linear_solver/linear_solver.h"

using namespace spnipu;
using namespace operations_research;

namespace {

// Helper function to extract edge information from the SPN
std::vector<std::pair<NodeRef, NodeRef>> extractEdges(SPN& spn) {
  std::vector<std::pair<NodeRef, NodeRef>> edges;
  std::unordered_set<NodeRef> visited;

  // Function to recursively extract edges
  std::function<void(NodeRef)> extractEdgesFromNode = [&](NodeRef node) {
    if (visited.count(node)) return;
    visited.insert(node);

    for (NodeRef child : node->getChildren()) {
      edges.push_back(
          {child, node});  // child is the predecessor, node is the successor
      extractEdgesFromNode(child);
    }
  };

  extractEdgesFromNode(spn.getRoot());
  return edges;
}

// A helper function to generate variable names.
std::string VarName(const std::string& base, int v, int p, int s) {
  std::ostringstream oss;
  oss << base << "_" << v << "_" << p << "_" << s;
  return oss.str();
}

class ILPScheduler {
 public:
  ILPScheduler(SPN& spn, unsigned numProcessors = 5, unsigned maxSupersteps = 5,
               unsigned g = 1, unsigned L = 10)
      : spn_(spn),
        numProcessors_(numProcessors),
        maxSupersteps_(maxSupersteps),
        g_(g),
        L_(L) {
    edges_ = extractEdges(spn);
  }

  std::optional<BSPSchedule> schedule() {
    // Extract all nodes from the SPN
    std::vector<NodeRef> nodes;
    spn_.walk([&nodes](NodeRef node) { nodes.push_back(node); });
    std::unordered_map<NodeRef, int> nodeToIndex;
    for (int i = 0; i < nodes.size(); i++) {
      nodeToIndex[nodes[i]] = i;
    }

    // Create the MPSolver instance.
    std::unique_ptr<MPSolver> solver;
    solver = std::make_unique<MPSolver>(
        "SPN-IPU-Scheduler", MPSolver::GUROBI_MIXED_INTEGER_PROGRAMMING);
    if (!solver) {
      spdlog::info("Gurobi solver not available. Falling back to CBC.");
      solver = std::make_unique<MPSolver>(
          "SPN-IPU-Scheduler", MPSolver::CBC_MIXED_INTEGER_PROGRAMMING);
      if (!solver) {
        throw std::runtime_error("Failed to create MPSolver instance.");
      }
    }

    const int n = nodes.size();
    const int P = numProcessors_;
    const int S = maxSupersteps_;  // Upper bound on the number of supersteps
    const double w = 10;           // Computation cost per node
    const double g = g_;           // Communication cost per unit
    const double L = L_;           // Latency cost per superstep

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
          comp[v][p][s] = solver->MakeIntVar(0, 1, VarName("comp", v, p, s));
          pres[v][p][s] = solver->MakeIntVar(0, 1, VarName("pres", v, p, s));
          sent[v][p][s] = solver->MakeIntVar(0, 1, VarName("sent", v, p, s));
          recv[v][p][s] = solver->MakeIntVar(0, 1, VarName("recv", v, p, s));
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
    // (This is a simplified version; in a full model one might need additional
    // terms to handle interprocessor communications.) Here we assume spn_
    // provides a list or way to iterate over edges.

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
      Work[s] = solver->MakeNumVar(0, solver->infinity(),
                                   "Work_" + std::to_string(s));
      for (int p = 0; p < P; p++) {
        LinearExpr work_p;
        for (int v = 0; v < n; v++) {
          work_p += LinearExpr(comp[v][p][s]) * w;
        }
        // Work[s] >= work_p for all processors.
        solver->MakeRowConstraint(Work[s] >= work_p);
      }
    }

    // (6) Define communication cost per superstep.
    std::vector<const MPVariable*> Comm(S, nullptr);
    for (int s = 0; s < S; s++) {
      Comm[s] = solver->MakeNumVar(0, solver->infinity(),
                                   "Comm_" + std::to_string(s));
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
      totalCost += LinearExpr(Work[s]) + g * LinearExpr(Comm[s]) + L;
    }
    MPObjective* const objective = solver->MutableObjective();
    objective->OptimizeLinearExpr(totalCost, false);

    // Set time limit
    solver->SetTimeLimit(absl::Minutes(10));

    // Export model to LP format (optional)
    // std::string model_str;
    // if (solver->ExportModelAsLpFormat(false, &model_str)) {
    //   spdlog::info("LP Model format:");
    //   spdlog::info("{}", model_str);
    // }

    // Solve the ILP
    spdlog::trace(
        "ILP problem constructed with {} variables and {} constraints",
        solver->NumVariables(), solver->NumConstraints());
    spdlog::info("Starting to solve the ILP Problem...");
    // solver->EnableOutput();
    const MPSolver::ResultStatus result_status = solver->Solve();

    // Create a BSP schedule based on the solution
    BSPSchedule schedule(spn_);

    if (result_status == MPSolver::OPTIMAL ||
        result_status == MPSolver::FEASIBLE) {
      spdlog::info("Solution ({}) found after {} s with objective value: {}",
                   ProtoEnumToString<MPSolverResponseStatus>(
                       static_cast<MPSolverResponseStatus>(result_status)),
                   absl::ToDoubleSeconds(solver->DurationSinceConstruction()),
                   objective->Value());

      // Extract the solution and create a BSP schedule
      for (NodeRef node : nodes) {
        int v = nodeToIndex[node];
        for (unsigned p = 0; p < numProcessors_; p++) {
          for (unsigned s = 0; s < maxSupersteps_; s++) {
            if (comp[v][p][s]->solution_value() > 0.5) {
              schedule.scheduleNode(node, s, p);
            }
          }
        }
      }

      schedule.lock();
      schedule.validate();
      spdlog::trace("Schedule validated");
      return tidySchedule(schedule);
    } else {
      spdlog::error("No solution found after {} s: {}",
                    absl::ToDoubleSeconds(solver->DurationSinceConstruction()),
                    ProtoEnumToString<MPSolverResponseStatus>(
                        static_cast<MPSolverResponseStatus>(result_status)));
      return std::nullopt;
    }
  }

  /// Removes empty supersteps from the schedule.
  BSPSchedule tidySchedule(BSPSchedule& schedule) {
    BSPSchedule newSchedule(spn_);
    unsigned shiftedSuperstep = 0;
    for (unsigned s = 0; s < maxSupersteps_; s++) {
      if (schedule.getNodesOfSuperstep(s).size() > 0) {
        for (auto& [proc, nodes] : schedule.getNodesOfSuperstep(s)) {
          for (NodeRef node : nodes) {
            newSchedule.scheduleNode(node, shiftedSuperstep, proc);
          }
        }
        shiftedSuperstep++;
      }
    }
    newSchedule.lock();
    return newSchedule;
  }

 private:
  SPN& spn_;
  unsigned numProcessors_;
  unsigned maxSupersteps_;
  unsigned g_;  // Communication cost per unit
  unsigned L_;  // Latency cost per superstep
  std::vector<std::pair<NodeRef, NodeRef>> edges_;
};
}  // namespace

std::optional<BSPSchedule> spnipu::scheduleWithILP(SPN& spn) {
  ILPScheduler scheduler(spn);
  return scheduler.schedule();
}