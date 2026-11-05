#include "robotics/controllers/manipulator/OperationalSpaceControl.hpp"

namespace controllers
{
    template class OperationalSpaceControl<float, 2, 2>;
    template class OperationalSpaceControl<float, 3, 2>;
    template class OperationalSpaceControl<float, 7, 6>;
}
