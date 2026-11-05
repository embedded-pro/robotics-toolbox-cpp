#include "robotics/planning/RrtConnect.hpp"

namespace planning
{
    template class RrtConnect<float, 2, 256>;
    template class RrtConnect<float, 2, 10>;
    template PlanStatus RrtConnect<float, 2, 256>::Plan<math::Vector<float, 2> (&)(), bool (&)(const math::Vector<float, 2>&)>(
        const math::Vector<float, 2>&, const math::Vector<float, 2>&, math::Vector<float, 2> (&)(), bool (&)(const math::Vector<float, 2>&));
    template PlanStatus RrtConnect<float, 2, 10>::Plan<math::Vector<float, 2> (&)(), bool (&)(const math::Vector<float, 2>&)>(
        const math::Vector<float, 2>&, const math::Vector<float, 2>&, math::Vector<float, 2> (&)(), bool (&)(const math::Vector<float, 2>&));
    template void RrtConnect<float, 2, 256>::Shortcut<float (&)(), bool (&)(const math::Vector<float, 2>&)>(std::size_t, float (&)(), bool (&)(const math::Vector<float, 2>&));
}
