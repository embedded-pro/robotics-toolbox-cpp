#include "robotics/dynamics/EffectiveJointInertia.hpp"

template std::optional<math::Vector<float, 2>> dynamics::EffectiveJointInertia<float, 2>(const math::SquareMatrix<float, 2>&);
template std::optional<math::Vector<float, 6>> dynamics::EffectiveJointInertia<float, 6>(const math::SquareMatrix<float, 6>&);
