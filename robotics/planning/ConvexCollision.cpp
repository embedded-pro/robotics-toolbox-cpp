#include "robotics/planning/ConvexCollision.hpp"

namespace planning
{
    template struct ConvexShape<float, 16>;
    template class ConvexCollision<float, 16, 64>;
    template class ConvexCollision<float, 16, 8>;
}
