#pragma once

#include <filesystem>

#include "libspnipu/model/SPN.hpp"

namespace spnipu {
SPN deserializeSPN(std::filesystem::path filename);
}  // namespace spnipu