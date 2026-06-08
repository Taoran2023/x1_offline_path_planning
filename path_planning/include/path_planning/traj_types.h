#pragma once
#include <Eigen/Core>
#include <vector>
#include <utility>

namespace path_planning {

enum TrajSegTag { SEG_GROUND = 0, SEG_AERIAL = 1, SEG_TRANS = 2 };

using TrajSegment = std::pair<TrajSegTag, std::vector<Eigen::Vector3d>>;
using AgentPath   = std::vector<TrajSegment>;

} // namespace path_planning
