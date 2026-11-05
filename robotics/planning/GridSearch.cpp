#include "robotics/planning/GridSearch.hpp"

namespace planning
{
    template class GridSearch<float, 8, 6>;
    template class GridSearch<float, 3, 3, 3>;
    template std::optional<float> GridSearch<float, 8, 6>::Search<std::optional<float> (&)(GridIndex)>(GridIndex, GridIndex, std::optional<float> (&)(GridIndex));
    template std::optional<float> GridSearch<float, 3, 3, 3>::Search<std::optional<float> (&)(GridIndex)>(GridIndex, GridIndex, std::optional<float> (&)(GridIndex));
}
