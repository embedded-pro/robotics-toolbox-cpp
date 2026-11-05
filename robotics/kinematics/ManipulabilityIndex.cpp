#include "robotics/kinematics/ManipulabilityIndex.hpp"

namespace kinematics
{
    template class ManipulabilityIndex<float, 3, 2>;
    template class ManipulabilityIndex<float, 2, 3>;
    template class ManipulabilityIndex<float, 6, 2>;
    template class ManipulabilityIndex<float, 6, 6>;
}
