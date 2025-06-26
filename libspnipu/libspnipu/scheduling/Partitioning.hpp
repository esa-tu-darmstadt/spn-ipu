#pragma once

#include <optional>

#include "libspnipu/model/Partitioning.hpp"

namespace spnipu {

std::optional<Partitioning> partitionWithDagP(const SPN &spn, size_t numParts);
}  // namespace spnipu