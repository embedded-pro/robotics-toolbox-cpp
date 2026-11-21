#include "robotics/dynamics/EffectiveJointInertia.hpp"

namespace dynamics
{
    template std::optional<math::Vector<float, 2>> EffectiveJointInertia<float, 2>(const math::SquareMatrix<float, 2>&);
    template std::optional<math::Vector<float, 6>> EffectiveJointInertia<float, 6>(const math::SquareMatrix<float, 6>&);
}
