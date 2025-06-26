#pragma once

#include "libspnipu/scheduling/BSPSchedule.hpp"
#include "poplar/Tensor.hpp"
namespace spnipu {

void lowerToGraphene(const BSPSchedule &schedule, bool verbose = false);

}