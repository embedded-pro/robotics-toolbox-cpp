#include "robotics/controllers/manipulator/CableTensionDistribution.hpp"

namespace controllers
{
    template class CableTensionDistribution<float, 3, 2>;
    template class CableTensionDistribution<float, 2, 2>;
    template class CableTensionDistribution<float, 8, 6>;
}
