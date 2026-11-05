#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "robotics/kinematics/ChainPoseKinematics.hpp"
#include "robotics/planning/PrimitiveDistance.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace planning
{
    template<typename T>
    struct SelfCollisionResult
    {
        static_assert(std::is_floating_point_v<T>, "SelfCollisionResult supports floating-point types");

        T minimumDistance{ std::numeric_limits<T>::infinity() };
        std::array<std::uint8_t, 2> pair{};
        bool colliding{ false };
        std::size_t checkedPairs{ 0 };
    };

    template<typename T, std::size_t NumLinks>
    class SelfCollision
    {
        static_assert(std::is_floating_point_v<T>, "SelfCollision supports floating-point types");
        static_assert(NumLinks >= 2 && NumLinks <= 32, "SelfCollision supports 2 to 32 links");

    public:
        using Frames = kinematics::FrameChain<T, NumLinks>;

        explicit SelfCollision(const std::array<T, NumLinks>& radii);

        void Allow(std::size_t i, std::size_t j);
        void Disallow(std::size_t i, std::size_t j);
        bool IsAllowed(std::size_t i, std::size_t j) const;

        OPTIMIZE_FOR_SPEED SelfCollisionResult<T> Check(const Frames& frames, T margin) const;
        Capsule<T> LinkCapsule(const Frames& frames, std::size_t link) const;

    private:
        static std::uint32_t Bit(std::size_t index);

        std::array<T, NumLinks> radii;
        std::array<std::uint32_t, NumLinks> allowed{};
    };

    template<typename T, std::size_t NumLinks>
    SelfCollision<T, NumLinks>::SelfCollision(const std::array<T, NumLinks>& radii)
        : radii{ radii }
    {
        for (std::size_t i = 0; i + 1 < NumLinks; ++i)
            Allow(i, i + 1);
    }

    template<typename T, std::size_t NumLinks>
    void SelfCollision<T, NumLinks>::Allow(std::size_t i, std::size_t j)
    {
        allowed[i] |= Bit(j);
        allowed[j] |= Bit(i);
    }

    template<typename T, std::size_t NumLinks>
    void SelfCollision<T, NumLinks>::Disallow(std::size_t i, std::size_t j)
    {
        allowed[i] &= ~Bit(j);
        allowed[j] &= ~Bit(i);
    }

    template<typename T, std::size_t NumLinks>
    bool SelfCollision<T, NumLinks>::IsAllowed(std::size_t i, std::size_t j) const
    {
        return (allowed[i] & Bit(j)) != 0;
    }

    template<typename T, std::size_t NumLinks>
    OPTIMIZE_FOR_SPEED SelfCollisionResult<T> SelfCollision<T, NumLinks>::Check(const Frames& frames, T margin) const
    {
        SelfCollisionResult<T> result{};
        for (std::size_t i = 0; i < NumLinks; ++i)
            for (std::size_t j = i + 1; j < NumLinks; ++j)
            {
                if (IsAllowed(i, j))
                    continue;

                const T distance{ PrimitiveDistance<T>::CapsuleCapsule(LinkCapsule(frames, i), LinkCapsule(frames, j)).distance };
                ++result.checkedPairs;
                if (distance < result.minimumDistance)
                {
                    result.minimumDistance = distance;
                    result.pair = { static_cast<std::uint8_t>(i), static_cast<std::uint8_t>(j) };
                }
            }

        result.colliding = result.minimumDistance < margin;
        return result;
    }

    template<typename T, std::size_t NumLinks>
    Capsule<T> SelfCollision<T, NumLinks>::LinkCapsule(const Frames& frames, std::size_t link) const
    {
        const auto& end{ link + 1 < NumLinks ? frames.jointOrigins[link + 1] : frames.tool.translation };
        return Capsule<T>{ Segment<T>{ frames.jointOrigins[link], end }, radii[link] };
    }

    template<typename T, std::size_t NumLinks>
    std::uint32_t SelfCollision<T, NumLinks>::Bit(std::size_t index)
    {
        return std::uint32_t{ 1 } << index;
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class SelfCollision<float, 4>;
    extern template class SelfCollision<float, 6>;
    extern template class SelfCollision<float, 7>;
#endif
}
