#include <spdlog/spdlog.h>

#include <unordered_map>

#include "dagP.h"
#include "libspnipu/scheduling/Partitioning.hpp"
#include "libspnipu/scheduling/PerformanceModel.hpp"

using namespace spnipu;

std::optional<Partitioning> spnipu::partitionWithDagP(const SPN &spn,
                                                      size_t numParts) {
  const auto &nodes = spn.getNodes();
  if (nodes.empty()) {
    spdlog::error("Cannot partition empty SPN");
    return std::nullopt;
  }

  idxType nVrtx = nodes.size();

  // Create node to vertex id mapping (1-based indexing for dagP)
  std::unordered_map<NodeRef, idxType> nodeToId;
  std::vector<NodeRef> nodeVec;
  idxType vertexId = 1;
  for (const auto &nodePtr : nodes) {
    NodeRef node = nodePtr.get();
    nodeToId[node] = vertexId++;
    nodeVec.push_back(node);
  }

  // Count edges by iterating through all nodes and their children
  idxType nEdge = 0;
  for (const auto &nodePtr : nodes) {
    nEdge += nodePtr->getChildren().size();
  }

  assert(nEdge > numParts &&
         "Number of edges must be greater than number of parts");

  dgraph graph;
  MLGP_option opt;
  if (int error = dagP_init_parameters(&opt, numParts)) {
    spdlog::error("Failed to initialize MLGP options with error code: {}",
                  error);
    return std::nullopt;
  }

  if (spdlog::get_level() <= spdlog::level::debug) {
    printOptions(&opt, ' ');
    opt.print = 1;
  }

  // Allocate graph data with vertex weights and edge costs
  allocateDGraphData(&graph, nVrtx, nEdge, DG_FRMT_VWEC);

  PerformanceModel perfModel(spn);

  // Set vertex weights using performance model
  for (idxType i = 1; i <= nVrtx; i++) {
    NodeRef node = nodeVec[i - 1];
    graph.vw[i] = perfModel.getComputationCost(node, 0);
  }

  // Build adjacency lists and edge weights
  std::vector<std::vector<idxType>> inEdges(nVrtx + 1);
  std::vector<std::vector<idxType>> outEdges(nVrtx + 1);
  std::vector<std::vector<ecType>> inCosts(nVrtx + 1);
  std::vector<std::vector<ecType>> outCosts(nVrtx + 1);

  ecType commCost = perfModel.getCommunicationCost();

  for (const auto &nodePtr : nodes) {
    NodeRef parent = nodePtr.get();
    idxType parentId = nodeToId[parent];

    for (NodeRef child : parent->getChildren()) {
      idxType childId = nodeToId[child];

      // Add edge from parent to child
      outEdges[parentId].push_back(childId);
      outCosts[parentId].push_back(commCost);

      // Add reverse edge for child's incoming edges
      inEdges[childId].push_back(parentId);
      inCosts[childId].push_back(commCost);
    }
  }

  // Populate dagP graph structure
  idxType edgeIdx = 0;
  for (idxType i = 1; i <= nVrtx; i++) {
    graph.inStart[i] = edgeIdx;
    for (size_t j = 0; j < inEdges[i].size(); j++) {
      graph.in[edgeIdx] = inEdges[i][j];
      graph.ecIn[edgeIdx] = inCosts[i][j];
      edgeIdx++;
    }
    graph.inEnd[i] = edgeIdx - 1;
  }

  edgeIdx = 0;
  for (idxType i = 1; i <= nVrtx; i++) {
    graph.outStart[i] = edgeIdx;
    for (size_t j = 0; j < outEdges[i].size(); j++) {
      graph.out[edgeIdx] = outEdges[i][j];
      graph.ecOut[edgeIdx] = outCosts[i][j];
      edgeIdx++;
    }
    graph.outEnd[i] = edgeIdx - 1;
  }

  // Find source nodes (nodes with no parents/incoming edges)
  graph.nbsources = sourcesList(&graph, graph.sources);

  // Find target nodes (nodes with no children/outgoing edges)
  graph.nbtargets = outputList(&graph, graph.targets);

  spdlog::debug("Found {} source nodes and {} target nodes", graph.nbsources,
                graph.nbtargets);

  // dagP uses 1-based indexing for parts!
  std::vector<idxType> parts(nVrtx + 1, 0);

  spdlog::debug("Partitioning SPN with {} vertices, {} edges into {} parts",
                nVrtx, nEdge, numParts);

  ecType edgeCut = dagP_partition_from_dgraph(&graph, &opt, parts.data());

  spdlog::info("Partitioning completed with edge cut: {}", edgeCut);

  // Create partitioning result
  Partitioning partitioning(spn);
  std::vector<PartitionRef> partitionRefs(numParts);
  for (size_t i = 0; i < numParts; i++) {
    partitionRefs[i] = partitioning.createPartition<Partition>();
  }

  // Assign nodes to partitions
  for (idxType i = 1; i <= nVrtx; i++) {
    NodeRef node = nodeVec[i - 1];
    idxType partId = parts[i];
    if (partId < 0 || partId >= static_cast<idxType>(numParts)) {
      spdlog::error("Invalid partition ID {} for node {}", partId, i);
      dagP_free_option(&opt);
      dagP_free_graph(&graph);
      return std::nullopt;
    }
    partitioning.addNodeToPartition(node, partitionRefs[partId]);
  }

  // Free the resources allocated by dagP
  dagP_free_option(&opt);
  dagP_free_graph(&graph);

  // Set partitioning algorithm description
  partitioning.setPartitioningAlgorithmDescription(
      fmt::format("DagP with {} parts, edge cut: {}", numParts, edgeCut));

  partitioning.lock();
  return partitioning;
}
