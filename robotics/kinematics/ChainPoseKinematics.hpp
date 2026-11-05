#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/SE3Transform.hpp"
#include "robotics/dynamics/JointLink.hpp"
#include <array>
#include <cassert>
#include <cstddef>
#include <type_traits>

namespace kinematics
{
    template<typename T, std::size_t N>
    struct FrameChain
    {
        static_assert(std::is_floating_point_v<T>, "FrameChain supports floating-point types");

        std::array<math::Vector3<T>, N> jointOrigins{};
        std::array<math::Vector3<T>, N> jointAxes{};
        std::array<dynamics::JointType, N> jointTypes{};
        math::SE3Transform<T> tool{};
    };

    template<typename T, std::size_t N>
    class ChainPoseKinematics
    {
        static_assert(std::is_floating_point_v<T>, "ChainPoseKinematics supports floating-point types");
        static_assert(N > 0, "Number of links must be positive");

    public:
        using JointVector = math::Vector<T, N>;
        using LinkArray = std::array<dynamics::JointLink<T>, N>;
        using Transform = math::SE3Transform<T>;

        explicit ChainPoseKinematics(const Transform& toolInLastLink);

        OPTIMIZE_FOR_SPEED FrameChain<T, N> Compute(const LinkArray& links, const JointVector& q) const;
        Transform ToolPose(const LinkArray& links, const JointVector& q) const;

    private:
        Transform toolInLastLink;
    };

    template<typename T, std::size_t N>
    ChainPoseKinematics<T, N>::ChainPoseKinematics(const Transform& toolInLastLink)
        : toolInLastLink{ toolInLastLink }
    {}

    template<typename T, std::size_t N>
    OPTIMIZE_FOR_SPEED FrameChain<T, N> ChainPoseKinematics<T, N>::Compute(const LinkArray& links, const JointVector& q) const
    {
        FrameChain<T, N> frames{};
        Transform frame{};

        for (std::size_t i = 0; i < N; ++i)
        {
            assert(dynamics::HasUnitJointAxis(links[i]));
            const auto joint{ links[i].JointTransform(q[i]) };
            frames.jointAxes[i] = frame.rotation * links[i].jointAxis;
            frames.jointTypes[i] = links[i].type;
            frame = frame * joint;
            frames.jointOrigins[i] = frame.translation;
        }

        frames.tool = frame * toolInLastLink;
        return frames;
    }

    template<typename T, std::size_t N>
    typename ChainPoseKinematics<T, N>::Transform ChainPoseKinematics<T, N>::ToolPose(const LinkArray& links, const JointVector& q) const
    {
        return Compute(links, q).tool;
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class ChainPoseKinematics<float, 2>;
    extern template class ChainPoseKinematics<float, 3>;
    extern template class ChainPoseKinematics<float, 6>;
#endif
}
