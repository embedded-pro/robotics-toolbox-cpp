#include "robotics/localization/MonteCarloLocalization.hpp"

namespace localization
{
    template class LikelihoodField<float, 40, 40>;
    template class LikelihoodField<float, 64, 64>;
    template class MonteCarloLocalization<float, 4, 40, 40>;
    template class MonteCarloLocalization<float, 27, 40, 40>;
    template class MonteCarloLocalization<float, 128, 64, 64>;
    template void MonteCarloLocalization<float, 4, 40, 40>::InitializeAround<float (&)()>(const math::SE2Transform<float>&, const math::Vector3<float>&, float (&)());
    template void MonteCarloLocalization<float, 4, 40, 40>::Predict<float (&)()>(const math::SE2Transform<float>&, float (&)());
    template void MonteCarloLocalization<float, 27, 40, 40>::Initialize<math::SE2Transform<float> (&)(std::size_t)>(math::SE2Transform<float> (&)(std::size_t));
    template bool MonteCarloLocalization<float, 27, 40, 40>::Update<float (&)()>(std::span<const RangeBeam<float>>, float (&)());
}
