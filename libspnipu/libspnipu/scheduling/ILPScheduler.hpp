#pragma once

#include "libspnipu/scheduling/BSPSchedule.hpp"

namespace spnipu {
std::optional<BSPSchedule> scheduleWithILP(SPN &spn);
}