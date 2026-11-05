#include "robotics/localization/OccupancyGridMap.hpp"

namespace localization
{
    template class OccupancyGridMap<float, 10, 10>;
    template class OccupancyGridMap<float, 64, 64>;
}
