#include "robotics/controllers/manipulator/HybridPositionForceControl.hpp"

namespace controllers
{
    template class HybridPositionForceControl<float, 2, 2>;
    template class HybridPositionForceControl<float, 6, 6>;
}
