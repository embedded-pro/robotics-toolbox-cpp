#include "robotics/kinematics/NearestSolution.hpp"

template std::optional<math::Vector<float, 2>> kinematics::WrapIntoLimits<float, 2>(const math::Vector<float, 2>&, const math::Vector<float, 2>&,
    const kinematics::JointLimitArray<float, 2>&);
template std::optional<math::Vector<float, 6>> kinematics::WrapIntoLimits<float, 6>(const math::Vector<float, 6>&, const math::Vector<float, 6>&,
    const kinematics::JointLimitArray<float, 6>&);
template kinematics::NearestSolutionResult<float, 2> kinematics::NearestSolution<float, 2, 3>(const std::array<std::optional<math::Vector<float, 2>>, 3>&,
    const math::Vector<float, 2>&, const kinematics::JointLimitArray<float, 2>&);
template kinematics::NearestSolutionResult<float, 6> kinematics::NearestSolution<float, 6, 8>(const std::array<std::optional<math::Vector<float, 6>>, 8>&,
    const math::Vector<float, 6>&, const kinematics::JointLimitArray<float, 6>&);
