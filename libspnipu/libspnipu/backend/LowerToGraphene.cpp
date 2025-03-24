#include "libspnipu/backend/LowerToGraphene.hpp"

#include <poplar/DataStream.hpp>
#include <popops/HostSliceTensor.hpp>

#include "libgraphene/common/Shape.hpp"
#include "libgraphene/common/Type.hpp"
#include "libgraphene/dsl/code/Operators.hpp"
#include "libgraphene/dsl/code/Value.hpp"
#include "libgraphene/dsl/tensor/Execute.hpp"
#include "libgraphene/dsl/tensor/Tensor.hpp"
#include "libgraphene/util/Context.hpp"
#include "libgraphene/util/Runtime.hpp"
#include "libspnipu/model/Nodes.hpp"
#include "libspnipu/model/Visitor.hpp"
#include "libspnipu/scheduling/BSPSchedule.hpp"

using namespace spnipu;
using namespace graphene;

namespace {
class TensorMapping {
 public:
  TensorMapping(unsigned superstep, const BSPSchedule &schedule)
      : schedule(schedule) {
    for (EdgeRef edge : schedule.getIncomingEdges(superstep)) {
      insertIncomingEdge(edge);
    }
    for (EdgeRef edge : schedule.getOutgoingEdges(superstep)) {
      insertOutgoingEdge(edge);
    }
    for (auto &[proc, nodes] : schedule.getNodesOfSuperstep(superstep)) {
      for (NodeRef node : nodes) {
        if (auto *leafNode = dynamic_cast<LeafNode *>(node)) {
          insertFeature(leafNode);
        }
      }
    }
    if (schedule.getSuperstep(schedule.getSPN().getRoot()) == superstep) {
      // Add the root node as an outgoing edge if it is scheduled in this
      // superstep
      insertOutgoingEdge({schedule.getSPN().getRoot(), nullptr});
    }
    calculateGlobalIndices();
  }

 private:
  void calculateGlobalIndices() {
    // calculate the global indices of the incoming edges
    unsigned globalInputTensorIndex = 0;
    for (auto &[proc, mapping] : incomingNodeToIndex) {
      // features are stored before the incoming edges
      globalInputTensorIndex += numFeaturesPerProc[proc];
      for (auto &[node, index] : mapping) {
        index.first = globalInputTensorIndex + index.second;
      }
      globalInputTensorIndex += numIncomingEdgesPerProc[proc];
    }

    // calculate the global indices of the features
    globalInputTensorIndex = 0;
    for (auto &[proc, mapping] : featureToIndex) {
      // features are stored before the incoming edges
      for (auto &[feature, index] : mapping) {
        index.first = globalInputTensorIndex + index.second;
      }
      globalInputTensorIndex += numFeaturesPerProc[proc];
      globalInputTensorIndex += numIncomingEdgesPerProc[proc];
    }

    // calculate the global indices of the outgoing edges
    unsigned globalOutputTensorIndex = 0;
    for (auto &[proc, mapping] : outgoingNodeToIndex) {
      for (auto &[node, index] : mapping) {
        index.first = globalOutputTensorIndex + index.second;
      }
      globalOutputTensorIndex += numOutgoingEdgesPerProc[proc];
    }
  }

  void insertIncomingEdge(EdgeRef edge) {
    unsigned proc = schedule.getProcessor(edge.getTarget());
    const auto &mapping = incomingNodeToIndex[proc];

    // Check if this proc already has an index for this node
    if (auto it = mapping.find(edge.getSource()); it != mapping.end()) {
      return;
    }

    // Insert a new index for this node
    unsigned nextIndex = numIncomingEdgesPerProc[proc]++;

    // the global index is calculated after all incoming edges have been
    // inserted
    incomingNodeToIndex[proc][edge.getSource()] = {0, nextIndex};
  }

  void insertOutgoingEdge(EdgeRef edge) {
    unsigned proc = schedule.getProcessor(edge.getSource());
    const auto &mapping = outgoingNodeToIndex[proc];

    // Check if this proc already has an index for this node
    if (auto it = mapping.find(edge.getSource()); it != mapping.end()) {
      return;
    }

    // Insert a new index for this node
    unsigned nextIndex = numOutgoingEdgesPerProc[proc]++;

    // the global index is calculated after all outgoing edges have been
    // inserted
    outgoingNodeToIndex[proc][edge.getSource()] = {0, nextIndex};
  }

  void insertFeature(LeafNode *leafNode) {
    unsigned proc = schedule.getProcessor(leafNode);
    unsigned globalFeatureIndex = leafNode->getScope();
    const auto &mapping = featureToIndex[proc];

    // Check if this proc already has an index for this feature
    if (auto it = mapping.find(globalFeatureIndex); it != mapping.end()) {
      return;
    }

    // Insert a new index for this feature
    unsigned nextIndex = numFeaturesPerProc[proc]++;
    featureToIndex[proc][globalFeatureIndex] = {0, nextIndex};
  }

 public:
  /// Returns the global index of the incoming edge in the input tensor.
  size_t getGlobalIndexOfIncomingEdge(EdgeRef edge) const {
    unsigned proc = schedule.getProcessor(edge.getTarget());
    return incomingNodeToIndex.at(proc).at(edge.getSource()).first;
  }

  /// Returns the processor-local index of the incoming edge in the input
  /// tensor. Input edges are stored after the features.
  size_t getLocalIndexOfIncomingEdge(EdgeRef edge) const {
    unsigned proc = schedule.getProcessor(edge.getTarget());
    unsigned edgeIndex =
        incomingNodeToIndex.at(proc).at(edge.getSource()).second;
    unsigned numLocalFeatures = numFeaturesPerProc.at(proc);
    return edgeIndex + numLocalFeatures;
  }

  /// Returns the global index of the outgoing edge in the output tensor.
  size_t getGlobalIndexOfOutgoingEdge(EdgeRef edge) const {
    unsigned proc = schedule.getProcessor(edge.getSource());
    return outgoingNodeToIndex.at(proc).at(edge.getSource()).first;
  }

  /// Returns the processor-local index of the outgoing edge in the output
  /// tensor.
  size_t getLocalIndexOfOutgoingEdge(EdgeRef edge) const {
    unsigned proc = schedule.getProcessor(edge.getSource());
    return outgoingNodeToIndex.at(proc).at(edge.getSource()).second;
  }

  /// Returns the global index of the global feature (aka scope) in the input
  /// tensor.
  size_t getGlobalIndexOfFeature(unsigned proc,
                                 unsigned globalFeatureIndex) const {
    return featureToIndex.at(proc).at(globalFeatureIndex).first;
  }

  /// Returns the processor-local index of the global feature (aka scope) in the
  /// input tensor. Features are stored before the input edges.
  size_t getLocalIndexOfFeature(unsigned proc,
                                unsigned globalFeatureIndex) const {
    return featureToIndex.at(proc).at(globalFeatureIndex).second;
  }

  /// Create the distributed shape of the input tensor. The input tensor
  /// consists of the features and the incoming edges, in this order.
  DistributedShape getIncomingShape() {
    FirstDimDistribution distr;
    distr.reserve(schedule.getNumProcessors());

    size_t numElements = 0;
    for (auto &[proc, numElementsOnProc] : numIncomingEdgesPerProc) {
      distr[proc] = numElementsOnProc;
      numElements += numElementsOnProc;
    }

    for (auto &[proc, numElementsOnProc] : numFeaturesPerProc) {
      distr[proc] += numElementsOnProc;
      numElements += numElementsOnProc;
    }

    TensorShape globalShape({numElements});
    return DistributedShape::onTiles(globalShape, distr);
  }

  /// Create the distributed shape of the output tensor. The output tensor
  /// consists of the outgoing edges.
  DistributedShape getOutgoingShape() {
    FirstDimDistribution distr;
    distr.reserve(schedule.getNumProcessors());

    size_t numElements = 0;
    for (auto &[proc, numElementsOnProc] : numOutgoingEdgesPerProc) {
      distr[proc] = numElementsOnProc;
      numElements += numElementsOnProc;
    }

    TensorShape globalShape({numElements});
    return DistributedShape::onTiles(globalShape, distr);
  }

 private:
  // For each processor, maps an incoming node to its global and
  // processor-local index in the input tensor
  std::map<unsigned, std::unordered_map<NodeRef, std::pair<unsigned, unsigned>>>
      incomingNodeToIndex;

  // For each processor, maps an outgoing node to its global and
  // processor-local index in the output tensor
  std::map<unsigned, std::unordered_map<NodeRef, std::pair<unsigned, unsigned>>>
      outgoingNodeToIndex;

  // For each processor, maps a global feature-index (aka scope) to its global
  // and processor-local index in the input tensor
  std::map<unsigned,
           std::unordered_map<unsigned, std::pair<unsigned, unsigned>>>
      featureToIndex;

  // Keeps track of the number of incoming and outgoing edges / features per
  // processor. These simply hold the currently largest processor-local index
  // per processor.
  std::map<unsigned, unsigned> numIncomingEdgesPerProc;
  std::map<unsigned, unsigned> numOutgoingEdgesPerProc;
  std::map<unsigned, unsigned> numFeaturesPerProc;

  const BSPSchedule &schedule;
};

struct LowerNodeVisitor : NodeVisitor {
  LowerNodeVisitor(std::unordered_map<Node *, codedsl::Value> &nodeToValue,
                   codedsl::Value &inputTensor, TensorMapping &tensorMapping,
                   unsigned proc)
      : nodeToValue(nodeToValue),
        inputTensor(inputTensor),
        tensorMapping(tensorMapping),
        proc(proc) {}

  void visit(ProductNode *node) final {
    if (nodeToValue.contains(node)) {
      return;
    }

    codedsl::Variable result(Type::FLOAT32, 1);

    for (NodeRef child : node->getChildren()) {
      child->accept(this);
      codedsl::Value &childValue = nodeToValue.at(child);
      result = result * childValue;
    }

    nodeToValue.emplace(
        std::make_pair<Node *, codedsl::Value>(node, std::move(result)));
  }
  void visit(SumNode *node) final {
    if (nodeToValue.contains(node)) {
      return;
    }

    codedsl::Variable result(Type::FLOAT32, 0);

    for (unsigned i = 0; i < node->getChildren().size(); i++) {
      NodeRef child = node->getChildren()[i];
      child->accept(this);
      codedsl::Value &childValue = nodeToValue.at(child);
      result = result + childValue * (float)node->getWeight(i);
    }

    nodeToValue.emplace(
        std::make_pair<Node *, codedsl::Value>(node, std::move(result)));
  }
  void visit(GaussianLeafNode *node) final {
    if (nodeToValue.contains(node)) {
      return;
    }

    uint32_t index =
        tensorMapping.getLocalIndexOfFeature(proc, node->getScope());
    codedsl::Value x = inputTensor[index];
    // Calculate Gaussian distribution using:
    // e^(-(x - mean)^2/2*variance))/sqrt(2*PI*variance)

    float variance = node->getVariance();
    float mean = node->getMean();

    codedsl::Variable result =
        codedsl::Exp(0 - ((x - mean) * (x - mean)) / (2 * variance)) /
        (float)sqrt(2 * M_PI * variance);

    nodeToValue.emplace(
        std::make_pair<Node *, codedsl::Value>(node, std::move(result)));
  }
  std::unordered_map<Node *, codedsl::Value> &nodeToValue;
  codedsl::Value &inputTensor;
  TensorMapping &tensorMapping;
  unsigned proc;
};

}  // namespace

void spnipu::lowerToGraphene(BSPSchedule &schedule) {
  schedule.lock();

  poplar::Graph &graph = Context::graph();
  Runtime &runtime = Runtime::instance();

  SPN &spn = schedule.getSPN();
  unsigned numFeatures = spn.getNumFeatures();

  TypeRef inputType = Type::FLOAT32;
  TypeRef outputType = Type::FLOAT32;
  TypeRef intermediateType = Type::FLOAT32;

  // Each superstep has an input and an output tensor. During computation, the
  // superstep computes the output tensor from the input tensor. After
  // computation, the output tensor is copied into a view of the input tensors
  // of the next supersteps. The input tensors consist of the features and the
  // incoming edges, and the output tensors consist of the outgoing edges.
  std::vector<Tensor> inputTensors, outputTensors;
  inputTensors.reserve(schedule.getNumSupersteps());
  outputTensors.reserve(schedule.getNumSupersteps());

  std::vector<TensorMapping> tensorMappings;
  tensorMappings.reserve(schedule.getNumSupersteps());

  for (unsigned i = 0; i < schedule.getNumSupersteps(); i++) {
    // Construct the tensor mapping for the current superstep
    TensorMapping &mapping = tensorMappings.emplace_back(i, schedule);

    // Create the input and output tensors for the current superstep
    inputTensors.emplace_back(
        Tensor::uninitialized(inputType, mapping.getIncomingShape()));
    outputTensors.emplace_back(
        Tensor::uninitialized(outputType, mapping.getOutgoingShape()));
  }

  // There is exactly one input stream and one output stream for the whole
  // program. The input stream contains the inputs of the SPN (the features),
  // and the output stream contains the output of the SPN (the joint
  // probability).
  poplar::DataStream inputStream = graph.addHostToDeviceFIFO(
      "inputStream", inputType->poplarType(), numFeatures);
  poplar::DataStream outputStream =
      graph.addDeviceToHostFIFO("outputStream", outputType->poplarType(), 1);

  // Copy the features from the input stream to the input tensors
  poplar::Tensor inputTensor = popops::createHostTransferableTensor(
      graph, inputType->poplarType(), {numFeatures}, false);
  Context::program().add(poplar::program::Copy(inputStream, inputTensor));
  spn.getRoot()->walk([&](NodeRef node) {
    if (auto *leafNode = dynamic_cast<LeafNode *>(node)) {
      unsigned scope = leafNode->getScope();
      unsigned proc = schedule.getProcessor(leafNode);
      unsigned superstep = schedule.getSuperstep(node);
      const TensorMapping &mapping = tensorMappings[superstep];
      unsigned index = mapping.getGlobalIndexOfFeature(proc, scope);
      poplar::Tensor srcFeatureTensor = inputTensor.slice(scope, scope + 1);
      poplar::Tensor featureValue =
          inputTensors[superstep].tensor().slice(index, index + 1);
      Context::program().add(
          poplar::program::Copy(srcFeatureTensor, featureValue));
    }
  });

  for (unsigned i = 0; i < schedule.getNumSupersteps(); i++) {
    TensorMapping &mapping = tensorMappings[i];

    const auto &nodesPerProcs = schedule.getNodesOfSuperstep(i);
    const auto &incomingEdges = schedule.getIncomingEdges(i);
    const auto &outgoingEdges = schedule.getOutgoingEdges(i);

    // Group the incoming and outgoing edges by processor
    std::unordered_map<unsigned, std::unordered_set<EdgeRef>>
        incomingEdgesPerProc;
    std::unordered_map<unsigned, std::unordered_set<EdgeRef>>
        outgoingEdgesPerProc;
    for (EdgeRef edge : incomingEdges) {
      incomingEdgesPerProc[schedule.getProcessor(edge.getTarget())].insert(
          edge);
    }
    for (EdgeRef edge : outgoingEdges) {
      outgoingEdgesPerProc[schedule.getProcessor(edge.getSource())].insert(
          edge);
    }
    // Add the root node as an outgoing edge if it is scheduled in this
    // superstep
    if (schedule.getSuperstep(spn.getRoot()) == i) {
      outgoingEdgesPerProc[schedule.getProcessor(spn.getRoot())].insert(
          {spn.getRoot(), nullptr});
    }

    for (auto &[proc, nodes] : nodesPerProcs) {
      using namespace codedsl;

      // Execute the nodes of the current superstep in parallel
      Context::ExecuteInParallel parallelContext;

      ExecuteOnSingleTile(
          [&](Value input, Value output) {
            // Mapping from a node to the value of its result.
            std::unordered_map<Node *, Value> nodeToValue;

            // Initialize with the input edges
            for (EdgeRef edge : incomingEdgesPerProc[proc]) {
              uint32_t index = mapping.getLocalIndexOfIncomingEdge(edge);
              nodeToValue.emplace(std::make_pair<Node *, Value>(
                  edge.getSource(), input[index]));
            }

            LowerNodeVisitor visitor(nodeToValue, input, mapping, proc);

            for (NodeRef node : nodes) {
              node->accept(&visitor);
            }

            // Copy the outgoing edges into the output tensor
            for (EdgeRef edge : outgoingEdgesPerProc[proc]) {
              uint32_t index = mapping.getLocalIndexOfOutgoingEdge(edge);
              output[index] = nodeToValue.at(edge.getSource());
            }
          },
          proc, In(inputTensors[i]), Out(outputTensors[i]));
    }

    // Copy the output tensor of the current superstep to the input tensors of
    // the next supersteps.
    spdlog::trace(
        "Copying output tensor of superstep {} to input tensors of the next "
        "supersteps:",
        i);
    for (EdgeRef edge : outgoingEdges) {
      // Copy this edge to the input tensor of the next superstep
      unsigned index = mapping.getGlobalIndexOfOutgoingEdge(edge);
      poplar::Tensor outputValue =
          outputTensors[i].tensor().slice(index, index + 1);
      unsigned targetSuperstep = schedule.getSuperstep(edge.getTarget());
      TensorMapping &targetMapping = tensorMappings[targetSuperstep];
      unsigned targetInputIndex =
          targetMapping.getGlobalIndexOfIncomingEdge(edge);
      poplar::Tensor nextInputValue =
          inputTensors[targetSuperstep].tensor().slice(targetInputIndex,
                                                       targetInputIndex + 1);
      Context::program().add(
          poplar::program::Copy(outputValue, nextInputValue));

      spdlog::trace(
          "Copying output of node {} from output tensor of superstep {} at "
          "index {} to input tensor of superstep {} at index {}",
          (void *)edge.getSource(), i, index, targetSuperstep,
          targetInputIndex);
    }
    if (schedule.getSuperstep(spn.getRoot()) == i) {
      // Copy the output of the root node to the output stream
      unsigned index =
          mapping.getGlobalIndexOfOutgoingEdge({spn.getRoot(), nullptr});
      poplar::Tensor outputValue =
          outputTensors[i].tensor().slice(index, index + 1);
      Context::program().add(poplar::program::Copy(outputValue, outputStream));
      spdlog::trace(
          "Copying output of root node from output tensor of superstep {} at "
          "index {} to output stream",
          i, index);
    }
  }
}