#include "robotics/kinematics/NearestSolution.hpp"

namespace kinematics
{
    template std::optional<math::Vector<float, 6>> WrapIntoLimits<float, 6>(const math::Vector<float, 6>&, const math::Vector<float, 6>&, const JointLimitArray<float, 6>&);
    template NearestSolutionResult<float, 6> NearestSolution<float, 6, 8>(const std::array<std::optional<math::Vector<float, 6>>, 8>&, const math::Vector<float, 6>&,
        const JointLimitArray<float, 6>&);
}
