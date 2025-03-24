#include "libspnipu/frontend/Deserialize.hpp"

#include <capnp/serialize.h>

#include "libspnipu/model/Nodes.hpp"
#include "spflow.capnp.h"

using namespace spnipu;
using namespace ::capnp;
namespace input = spnipu::capnp;

namespace {

NodeRef createNode(input::Node::Reader node, SPN& spn) {
  if (node.isGaussian()) {
    auto gaussian = node.getGaussian();
    return spn.createNode<GaussianLeafNode>(
        gaussian.getMean(), gaussian.getStddev(), gaussian.getScope());
  } else if (node.isSum()) {
    return spn.createNode<SumNode>();
  } else if (node.isProduct()) {
    return spn.createNode<ProductNode>();
  } else if (node.isHist()) {
    auto hist = node.getHist();
    // FIXME
    return spn.createNode<GaussianLeafNode>(0, 0, hist.getScope());
  } else {
    throw std::runtime_error("Unsupported node type: " +
                             std::to_string(node.which()));
  }
}

void connectNodeWithChildren(NodeRef node, input::Node::Reader nodeReader,
                             std::unordered_map<int, NodeRef>& idToNode) {
  if (nodeReader.isSum()) {
    auto sum = nodeReader.getSum();
    for (size_t i = 0; i < sum.getChildren().size(); i++) {
      SumNode* sumNode = dynamic_cast<SumNode*>(node);
      int childID = sum.getChildren()[i];
      auto childNode = idToNode[childID];
      double weight = sum.getWeights()[i];
      sumNode->addSummand(childNode, weight);
    }
  } else if (nodeReader.isProduct()) {
    auto product = nodeReader.getProduct();
    for (int childID : product.getChildren()) {
      auto childNode = idToNode[childID];
      ProductNode* productNode = dynamic_cast<ProductNode*>(node);
      productNode->addFactor(childNode);
    }
  } else if (nodeReader.isGaussian() || nodeReader.isHist() ||
             nodeReader.hasCategorical()) {
    // Nothing to do
  } else {
    throw std::runtime_error("Unsupported node type: " +
                             std::to_string(nodeReader.which()));
  }
}
}  // namespace

// Some spdlog (fmt) formatters for capnp types. Helps with debugging.
template <>
struct ::fmt::formatter<::capnp::Text::Reader> {
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    return ctx.end();
  }
  template <typename FormatContext>
  auto format(const ::capnp::Text::Reader& input,
              FormatContext& ctx) -> decltype(ctx.out()) {
    // Cant we write the string directly, without copying?
    return format_to(ctx.out(), "{}", std::string(input.begin(), input.end()));
  }
};
template <>
struct ::fmt::formatter<::kj::String> {
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    return ctx.end();
  }
  template <typename FormatContext>
  auto format(const ::kj::String& input,
              FormatContext& ctx) -> decltype(ctx.out()) {
    // Cant we write the string directly, without copying?
    return format_to(ctx.out(), "{}", std::string(input.begin(), input.end()));
  }
};
template <>
struct ::fmt::formatter<::kj::StringTree> {
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    return ctx.end();
  }
  template <typename FormatContext>
  auto format(const ::kj::StringTree& input,
              FormatContext& ctx) -> decltype(ctx.out()) {
    // This one hurts. No really, it does.
    std::string result;
    input.visit([&result](::kj::ArrayPtr<const char> array) {
      result += std::string(array.begin(), array.end());
    });
    return format_to(ctx.out(), "{}", result);
  }
};

SPN spnipu::deserializeSPN(std::filesystem::path filename) {
  int fd = open(filename.c_str(), O_RDONLY);

  spdlog::debug("Deserializing SPN from file \"{}\"", filename.string());

  StreamFdMessageReader message(fd);
  SPN spn;

  auto header = message.getRoot<input::Header>();

  if (header.isModel())
    throw std::runtime_error("Should not deserialize a model, only a query");

  auto query = header.getQuery();
  if (!query.hasJoint())
    throw std::runtime_error("Can only deserialize joint queries");

  auto joint = query.getJoint();

  auto model = joint.getModel();
  if (model.hasName()) spdlog::debug("SPN Name: {}", model.getName());
  spdlog::debug("SPN as string: {}", model.toString());
  spdlog::debug("Num features: {}", model.getNumFeatures());
  spdlog::debug("Num nodes: {}", model.getNodes().size());

  // In the capnpproto format, nodes reference each other by id. We need to
  // create the nodes first, and then
  // connect them.

  // Create nodes
  std::unordered_map<int, NodeRef> idToNode;
  for (input::Node::Reader node : model.getNodes()) {
    auto id = node.getId();
    idToNode[id] = createNode(node, spn);
  }

  // Connect nodes
  for (auto node : model.getNodes()) {
    connectNodeWithChildren(idToNode[node.getId()], node, idToNode);
  }

  close(fd);

  spn.setRoot(idToNode[model.getRootNode()]);
  return spn;
}