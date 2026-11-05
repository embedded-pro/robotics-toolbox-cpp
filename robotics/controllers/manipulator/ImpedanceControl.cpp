#include "robotics/controllers/manipulator/ImpedanceControl.hpp"

namespace controllers
{
    template class ImpedanceControl<float, 2, 2>;
    template class ImpedanceControl<float, 6, 6>;
}
