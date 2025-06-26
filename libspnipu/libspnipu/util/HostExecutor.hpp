#pragma once

#include <spdlog/spdlog.h>

#include <cmath>
#include <vector>

#include "libspnipu/model/SPN.hpp"
#include "libspnipu/model/Visitor.hpp"

namespace spnipu {

class HostExecutor : public NodeVisitor {
 private:
  const SPN& spn_;
  std::vector<double> input_;
  double result_;
  bool verbose_;

 public:
  explicit HostExecutor(const SPN& spn, bool verbose = false)
      : spn_(spn), result_(0.0), verbose_(verbose) {}

  double evaluate(const std::vector<double>& input) {
    input_ = input;

    // Visit the root node to evaluate the SPN
    spn_.getRoot()->accept(this);
    return result_;
  }

  // NodeVisitor implementation
  void visit(ProductNode* node) override {
    double product = 1.0;
    for (const auto& child : node->getChildren()) {
      child->accept(this);
      product *= result_;
    }
    result_ = product;

    if (verbose_) spdlog::info("Evaluating ProductNode: result={}", result_);
  }

  void visit(SumNode* node) override {
    double sum = 0.0;
    const auto& children = node->getChildren();
    for (size_t i = 0; i < children.size(); ++i) {
      children[i]->accept(this);
      sum += result_ * node->getWeight(i);
    }
    result_ = sum;

    if (verbose_) spdlog::info("Evaluating SumNode: result={}", result_);
  }

  void visit(GaussianLeafNode* node) override {
    const unsigned scope = node->getScope();
    if (scope >= input_.size()) {
      throw std::out_of_range(fmt::format(
          "Scope {} is out of range for input size {}", scope, input_.size()));
    }

    const double x = input_[scope];
    const double mean = node->getMean();
    const double variance = node->getVariance();

    // Gaussian PDF: (1/sqrt(2πσ²)) * exp(-(x-μ)²/(2σ²))
    const double diff = x - mean;
    const double exponent = -(diff * diff) / (2.0 * variance);
    const double normalization = 1.0 / std::sqrt(2.0 * M_PI * variance);

    result_ = normalization * std::exp(exponent);

    if (verbose_)
      spdlog::info(
          "Evaluating GaussianLeafNode: x={}, mean={}, variance={}, result={}",
          x, mean, variance, result_);
  }
};

}  // namespace spnipu