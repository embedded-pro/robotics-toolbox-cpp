#include "robotics/trajectory/CartesianSlerpInterpolation.hpp"
#include "numerical/motion_profiles/PolynomialProfile.hpp"

namespace trajectory
{
    template class CartesianSlerpInterpolation<float, motion_profiles::PolynomialProfile<float>>;
}
