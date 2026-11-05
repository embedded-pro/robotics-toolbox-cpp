#pragma once

#include "numerical/math/Matrix.hpp"
#include <cstddef>
#include <type_traits>

namespace trajectory
{
    template<typename T, std::size_t Dof>
    struct JointTrajectoryState
    {
        static_assert(std::is_floating_point_v<T>, "JointTrajectoryState supports floating-point types");

        math::Vector<T, Dof> position{};
        math::Vector<T, Dof> velocity{};
        math::Vector<T, Dof> acceleration{};
    };
}
