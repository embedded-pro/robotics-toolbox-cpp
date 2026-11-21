#include "robotics/dynamics/JointLink.hpp"

namespace dynamics
{
    template struct JointLink<float>;
    template math::SquareMatrix<float, 3> ParallelAxisInertia<float>(float, const math::Vector<float, 3>&);
    template JointLink<float> AttachRigidBody<float>(JointLink<float>, float, const math::Vector<float, 3>&, const math::SquareMatrix<float, 3>&);
}
