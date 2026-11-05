#pragma once

#include "numerical/math/Matrix.hpp"
#include "numerical/math/SE3Transform.hpp"
#include <cstddef>
#include <type_traits>

namespace controllers
{
    template<typename T, std::size_t TaskDim>
    math::Vector<T, TaskDim> TaskError(const math::SE3Transform<T>& desired, const math::SE3Transform<T>& current)
    {
        static_assert(std::is_floating_point_v<T>, "TaskError supports floating-point types");
        static_assert(TaskDim == 6 || (TaskDim > 0 && TaskDim <= 3), "TaskError supports pose (6) or position (1 to 3) tasks");

        if constexpr (TaskDim == 6)
            return math::SE3Transform<T>::PoseError(desired, current);
        else
        {
            math::Vector<T, TaskDim> error{};
            for (std::size_t i = 0; i < TaskDim; ++i)
                error[i] = desired.translation[i] - current.translation[i];
            return error;
        }
    }
}
