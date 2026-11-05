#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <numbers>
#include <optional>
#include <span>
#include <type_traits>
#include <utility>

namespace planning
{
    enum class Connectivity : std::uint8_t
    {
        Faces,
        Full
    };

    enum class SearchMode : std::uint8_t
    {
        AStar,
        Dijkstra
    };

    struct GridIndex
    {
        std::uint16_t x{};
        std::uint16_t y{};
        std::uint16_t z{};

        friend bool operator==(const GridIndex&, const GridIndex&) = default;
    };

    template<typename T>
    struct GridSearchParameters
    {
        static_assert(std::is_floating_point_v<T>, "GridSearchParameters supports floating-point types");

        Connectivity connectivity{ Connectivity::Full };
        SearchMode mode{ SearchMode::AStar };
        bool allowCornerCutting{ false };
        T minimumCellCost{ 1 };
    };

    template<typename T, std::size_t Width, std::size_t Height, std::size_t Depth = 1>
    class GridSearch
    {
        static_assert(std::is_floating_point_v<T>, "GridSearch supports floating-point types");
        static_assert(Width > 0 && Height > 0 && Depth > 0, "Grid dimensions must be positive");
        static_assert(Width <= 65535 && Height <= 65535 && Depth <= 65535, "GridIndex coordinates are 16-bit");

    public:
        static constexpr std::size_t Cells{ Width * Height * Depth };

        using Index = std::conditional_t<(Cells < 65535), std::uint16_t, std::uint32_t>;

        static_assert(Cells < std::numeric_limits<std::uint32_t>::max(), "Grid too large for 32-bit cell indices");

        explicit GridSearch(const GridSearchParameters<T>& parameters);

        template<class CellCost>
        OPTIMIZE_FOR_SPEED std::optional<T> Search(GridIndex start, GridIndex target, CellCost&& cost);

        std::size_t ExtractPath(std::span<GridIndex> out) const;
        std::size_t Expanded() const;

    private:
        enum class CellState : std::uint8_t
        {
            Unseen,
            Open,
            Closed
        };

        struct Move
        {
            int dx;
            int dy;
            int dz;

            std::size_t Axes() const;
            unsigned Mask() const;
        };

        static constexpr Index none{ std::numeric_limits<Index>::max() };
        static constexpr std::array<T, 4> stepLengths{ T{}, T{ 1 }, std::numbers::sqrt2_v<T>, std::numbers::sqrt3_v<T> };

        static bool Inside(GridIndex cell);
        static Index Linear(GridIndex cell);
        static GridIndex Coordinates(Index cell);
        static std::optional<GridIndex> Offset(GridIndex cell, const Move& move);

        template<class CellCost>
        void Expand(Index current, CellCost& cost);
        template<class CellCost>
        void TryMove(Index current, GridIndex cell, const Move& move, CellCost& cost);
        template<class CellCost>
        bool CornerBlocked(GridIndex cell, const Move& move, CellCost& cost) const;
        void Relax(Index current, Index next, T candidate);

        T Heuristic(Index cell) const;
        bool Before(Index lhs, Index rhs) const;
        void Push(Index cell);
        Index Pop();
        void SiftUp(std::size_t slot);
        void SiftDown(std::size_t slot);
        void Place(std::size_t slot, Index cell);

        GridSearchParameters<T> parameters;
        std::array<T, Cells> costFromStart{};
        std::array<Index, Cells> parent{};
        std::array<Index, Cells> heap{};
        std::array<Index, Cells> heapSlot{};
        std::array<CellState, Cells> state{};
        std::size_t heapSize{ 0 };
        std::size_t expanded{ 0 };
        GridIndex goal{};
        Index goalCell{ none };
        bool found{ false };
    };

    template<typename T, std::size_t Width, std::size_t Height, std::size_t Depth>
    GridSearch<T, Width, Height, Depth>::GridSearch(const GridSearchParameters<T>& parameters)
        : parameters{ parameters }
    {}

    template<typename T, std::size_t Width, std::size_t Height, std::size_t Depth>
    template<class CellCost>
    OPTIMIZE_FOR_SPEED std::optional<T> GridSearch<T, Width, Height, Depth>::Search(GridIndex start, GridIndex target, CellCost&& cost)
    {
        found = false;
        expanded = 0;
        heapSize = 0;
        if (!Inside(start) || !Inside(target) || !cost(start) || !cost(target))
            return std::nullopt;

        state.fill(CellState::Unseen);
        goal = target;
        goalCell = Linear(target);
        const Index startCell{ Linear(start) };
        costFromStart[startCell] = T{};
        parent[startCell] = none;
        Push(startCell);

        while (heapSize > 0)
        {
            const Index current{ Pop() };
            state[current] = CellState::Closed;
            ++expanded;
            if (current == goalCell)
            {
                found = true;
                return costFromStart[current];
            }
            Expand(current, cost);
        }

        return std::nullopt;
    }

    template<typename T, std::size_t Width, std::size_t Height, std::size_t Depth>
    std::size_t GridSearch<T, Width, Height, Depth>::ExtractPath(std::span<GridIndex> out) const
    {
        if (!found)
            return 0;

        std::size_t length{ 0 };
        for (Index cell = goalCell; cell != none; cell = parent[cell])
            ++length;
        if (length > out.size())
            return 0;

        std::size_t slot{ length };
        for (Index cell = goalCell; cell != none; cell = parent[cell])
            out[--slot] = Coordinates(cell);
        return length;
    }

    template<typename T, std::size_t Width, std::size_t Height, std::size_t Depth>
    std::size_t GridSearch<T, Width, Height, Depth>::Expanded() const
    {
        return expanded;
    }

    template<typename T, std::size_t Width, std::size_t Height, std::size_t Depth>
    std::size_t GridSearch<T, Width, Height, Depth>::Move::Axes() const
    {
        return static_cast<std::size_t>(dx != 0) + static_cast<std::size_t>(dy != 0) + static_cast<std::size_t>(dz != 0);
    }

    template<typename T, std::size_t Width, std::size_t Height, std::size_t Depth>
    unsigned GridSearch<T, Width, Height, Depth>::Move::Mask() const
    {
        return (dx != 0 ? 1u : 0u) | (dy != 0 ? 2u : 0u) | (dz != 0 ? 4u : 0u);
    }

    template<typename T, std::size_t Width, std::size_t Height, std::size_t Depth>
    bool GridSearch<T, Width, Height, Depth>::Inside(GridIndex cell)
    {
        return cell.x < Width && cell.y < Height && cell.z < Depth;
    }

    template<typename T, std::size_t Width, std::size_t Height, std::size_t Depth>
    auto GridSearch<T, Width, Height, Depth>::Linear(GridIndex cell) -> Index
    {
        return static_cast<Index>(cell.x + Width * (cell.y + Height * cell.z));
    }

    template<typename T, std::size_t Width, std::size_t Height, std::size_t Depth>
    GridIndex GridSearch<T, Width, Height, Depth>::Coordinates(Index cell)
    {
        return GridIndex{ static_cast<std::uint16_t>(cell % Width), static_cast<std::uint16_t>((cell / Width) % Height), static_cast<std::uint16_t>(cell / (Width * Height)) };
    }

    template<typename T, std::size_t Width, std::size_t Height, std::size_t Depth>
    std::optional<GridIndex> GridSearch<T, Width, Height, Depth>::Offset(GridIndex cell, const Move& move)
    {
        const int x{ cell.x + move.dx };
        const int y{ cell.y + move.dy };
        const int z{ cell.z + move.dz };
        if (x < 0 || y < 0 || z < 0 || x >= static_cast<int>(Width) || y >= static_cast<int>(Height) || z >= static_cast<int>(Depth))
            return std::nullopt;

        return GridIndex{ static_cast<std::uint16_t>(x), static_cast<std::uint16_t>(y), static_cast<std::uint16_t>(z) };
    }

    template<typename T, std::size_t Width, std::size_t Height, std::size_t Depth>
    template<class CellCost>
    void GridSearch<T, Width, Height, Depth>::Expand(Index current, CellCost& cost)
    {
        constexpr int depthReach{ Depth > 1 ? 1 : 0 };
        const GridIndex cell{ Coordinates(current) };

        for (int dz = -depthReach; dz <= depthReach; ++dz)
            for (int dy = -1; dy <= 1; ++dy)
                for (int dx = -1; dx <= 1; ++dx)
                    TryMove(current, cell, Move{ dx, dy, dz }, cost);
    }

    template<typename T, std::size_t Width, std::size_t Height, std::size_t Depth>
    template<class CellCost>
    void GridSearch<T, Width, Height, Depth>::TryMove(Index current, GridIndex cell, const Move& move, CellCost& cost)
    {
        const std::size_t axes{ move.Axes() };
        if (axes == 0 || (parameters.connectivity == Connectivity::Faces && axes > 1))
            return;

        const auto neighbour{ Offset(cell, move) };
        if (!neighbour || state[Linear(*neighbour)] == CellState::Closed)
            return;

        const auto cellCost{ cost(*neighbour) };
        if (!cellCost || (axes > 1 && !parameters.allowCornerCutting && CornerBlocked(cell, move, cost)))
            return;

        Relax(current, Linear(*neighbour), costFromStart[current] + stepLengths[axes] * *cellCost);
    }

    template<typename T, std::size_t Width, std::size_t Height, std::size_t Depth>
    template<class CellCost>
    bool GridSearch<T, Width, Height, Depth>::CornerBlocked(GridIndex cell, const Move& move, CellCost& cost) const
    {
        const unsigned mask{ move.Mask() };
        for (unsigned subset = 1; subset < mask; ++subset)
        {
            if ((subset & ~mask) != 0u)
                continue;

            const auto side{ Offset(cell, Move{ (subset & 1u) != 0u ? move.dx : 0, (subset & 2u) != 0u ? move.dy : 0, (subset & 4u) != 0u ? move.dz : 0 }) };
            if (side && !cost(*side))
                return true;
        }

        return false;
    }

    template<typename T, std::size_t Width, std::size_t Height, std::size_t Depth>
    void GridSearch<T, Width, Height, Depth>::Relax(Index current, Index next, T candidate)
    {
        if (state[next] == CellState::Unseen)
        {
            costFromStart[next] = candidate;
            parent[next] = current;
            Push(next);
        }
        else if (candidate < costFromStart[next])
        {
            costFromStart[next] = candidate;
            parent[next] = current;
            SiftUp(heapSlot[next]);
        }
    }

    template<typename T, std::size_t Width, std::size_t Height, std::size_t Depth>
    T GridSearch<T, Width, Height, Depth>::Heuristic(Index cell) const
    {
        if (parameters.mode == SearchMode::Dijkstra)
            return T{};

        const GridIndex position{ Coordinates(cell) };
        int largest{ std::abs(position.x - goal.x) };
        int middle{ std::abs(position.y - goal.y) };
        int smallest{ std::abs(position.z - goal.z) };
        if (largest < middle)
            std::swap(largest, middle);
        if (middle < smallest)
            std::swap(middle, smallest);
        if (largest < middle)
            std::swap(largest, middle);

        if (parameters.connectivity == Connectivity::Faces)
            return parameters.minimumCellCost * static_cast<T>(largest + middle + smallest);

        return parameters.minimumCellCost * (static_cast<T>(largest) + (stepLengths[2] - stepLengths[1]) * static_cast<T>(middle) + (stepLengths[3] - stepLengths[2]) * static_cast<T>(smallest));
    }

    template<typename T, std::size_t Width, std::size_t Height, std::size_t Depth>
    bool GridSearch<T, Width, Height, Depth>::Before(Index lhs, Index rhs) const
    {
        const T lhsEstimate{ costFromStart[lhs] + Heuristic(lhs) };
        const T rhsEstimate{ costFromStart[rhs] + Heuristic(rhs) };
        if (lhsEstimate < rhsEstimate)
            return true;
        if (rhsEstimate < lhsEstimate)
            return false;

        return costFromStart[lhs] > costFromStart[rhs];
    }

    template<typename T, std::size_t Width, std::size_t Height, std::size_t Depth>
    void GridSearch<T, Width, Height, Depth>::Push(Index cell)
    {
        state[cell] = CellState::Open;
        Place(heapSize, cell);
        ++heapSize;
        SiftUp(heapSize - 1);
    }

    template<typename T, std::size_t Width, std::size_t Height, std::size_t Depth>
    auto GridSearch<T, Width, Height, Depth>::Pop() -> Index
    {
        const Index top{ heap[0] };
        --heapSize;
        if (heapSize > 0)
        {
            Place(0, heap[heapSize]);
            SiftDown(0);
        }

        return top;
    }

    template<typename T, std::size_t Width, std::size_t Height, std::size_t Depth>
    void GridSearch<T, Width, Height, Depth>::SiftUp(std::size_t slot)
    {
        const Index cell{ heap[slot] };
        while (slot > 0)
        {
            const std::size_t above{ (slot - 1) / 2 };
            if (!Before(cell, heap[above]))
                break;

            Place(slot, heap[above]);
            slot = above;
        }

        Place(slot, cell);
    }

    template<typename T, std::size_t Width, std::size_t Height, std::size_t Depth>
    void GridSearch<T, Width, Height, Depth>::SiftDown(std::size_t slot)
    {
        const Index cell{ heap[slot] };
        for (std::size_t child = 2 * slot + 1; child < heapSize; child = 2 * slot + 1)
        {
            if (child + 1 < heapSize && Before(heap[child + 1], heap[child]))
                ++child;
            if (!Before(heap[child], cell))
                break;

            Place(slot, heap[child]);
            slot = child;
        }

        Place(slot, cell);
    }

    template<typename T, std::size_t Width, std::size_t Height, std::size_t Depth>
    void GridSearch<T, Width, Height, Depth>::Place(std::size_t slot, Index cell)
    {
        heap[slot] = cell;
        heapSlot[cell] = static_cast<Index>(slot);
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class GridSearch<float, 8, 6>;
    extern template class GridSearch<float, 3, 3, 3>;
    extern template std::optional<float> GridSearch<float, 8, 6>::Search<std::optional<float> (&)(GridIndex)>(GridIndex, GridIndex, std::optional<float> (&)(GridIndex));
    extern template std::optional<float> GridSearch<float, 3, 3, 3>::Search<std::optional<float> (&)(GridIndex)>(GridIndex, GridIndex, std::optional<float> (&)(GridIndex));
#endif
}
