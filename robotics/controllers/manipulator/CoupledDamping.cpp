#include "robotics/controllers/manipulator/CoupledDamping.hpp"

namespace controllers
{
    template math::SquareMatrix<float, 2> CoupledDamping<float, 2>(const math::SquareMatrix<float, 2>&, const math::Vector<float, 2>&, float);
    template math::SquareMatrix<float, 6> CoupledDamping<float, 6>(const math::SquareMatrix<float, 6>&, const math::Vector<float, 6>&, float);
}
