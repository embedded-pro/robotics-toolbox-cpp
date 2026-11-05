#include "robotics/kinematics/PoseInverseKinematics.hpp"

namespace kinematics
{
    template class PoseIkProblem<float, 6>;
    template class PoseIkProblem<float, 7>;
    template class ManipulabilityDamping<float, 6>;
    template class ManipulabilityDamping<float, 7>;
    template class PoseInverseKinematics<float, 6>;
    template class PoseInverseKinematics<float, 7>;
}
