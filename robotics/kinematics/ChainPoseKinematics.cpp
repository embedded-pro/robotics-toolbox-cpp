#include "robotics/kinematics/ChainPoseKinematics.hpp"

namespace kinematics
{
    template class ChainPoseKinematics<float, 2>;
    template class ChainPoseKinematics<float, 3>;
    template class ChainPoseKinematics<float, 6>;
}
