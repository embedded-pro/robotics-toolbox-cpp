#include "robotics/mobile/PlanarOdometry.hpp"

namespace mobile
{
    template class PlanarOdometry<float>;
    template math::Vector3<float> IncrementDeviation<float>(const math::SE2Transform<float>&, const OdometryNoise<float>&);
}
