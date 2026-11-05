#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/SE3Transform.hpp"
#include "robotics/dynamics/JointLink.hpp"
#include "robotics/kinematics/ChainPoseKinematics.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace kinematics
{
    enum class DhConvention : std::uint8_t
    {
        Standard,
        Modified
    };

    template<typename T>
    struct DhLink
    {
        static_assert(std::is_floating_point_v<T>, "DhLink supports floating-point types");

        T a{};
        T alpha{};
        T d{};
        T theta{};
        dynamics::JointType type{ dynamics::JointType::Revolute };
    };

    template<typename T, std::size_t N>
    class DenavitHartenberg
    {
        static_assert(std::is_floating_point_v<T>, "DenavitHartenberg supports floating-point types");
        static_assert(N > 0, "Number of links must be positive");

    public:
        using JointVector = math::Vector<T, N>;
        using Transform = math::SE3Transform<T>;
        using LinkTable = std::array<DhLink<T>, N>;

        DenavitHartenberg(const LinkTable& links, DhConvention convention, const Transform& tool = Transform::Identity());

        OPTIMIZE_FOR_SPEED Transform LinkTransform(std::size_t i, T q) const;
        OPTIMIZE_FOR_SPEED Transform Forward(const JointVector& q) const;
        FrameChain<T, N> Frames(const JointVector& q) const;

    private:
        Transform StandardTransform(std::size_t i, T cosTheta, T sinTheta, T d) const;
        Transform ModifiedTransform(std::size_t i, T cosTheta, T sinTheta, T d) const;

        LinkTable links;
        DhConvention convention;
        Transform tool;
        std::array<T, N> cosAlpha{};
        std::array<T, N> sinAlpha{};
    };

    template<typename T, std::size_t N>
    DenavitHartenberg<T, N>::DenavitHartenberg(const LinkTable& links, DhConvention convention, const Transform& tool)
        : links{ links }
        , convention{ convention }
        , tool{ tool }
    {
        for (std::size_t i = 0; i < N; ++i)
        {
            cosAlpha[i] = math::Cos(links[i].alpha);
            sinAlpha[i] = math::Sin(links[i].alpha);
        }
    }

    template<typename T, std::size_t N>
    OPTIMIZE_FOR_SPEED typename DenavitHartenberg<T, N>::Transform DenavitHartenberg<T, N>::LinkTransform(std::size_t i, T q) const
    {
        const auto& link{ links[i] };
        const bool revolute{ link.type == dynamics::JointType::Revolute };
        const T theta{ link.theta + (revolute ? q : T{}) };
        const T d{ link.d + (revolute ? T{} : q) };

        if (convention == DhConvention::Standard)
            return StandardTransform(i, math::Cos(theta), math::Sin(theta), d);

        return ModifiedTransform(i, math::Cos(theta), math::Sin(theta), d);
    }

    template<typename T, std::size_t N>
    OPTIMIZE_FOR_SPEED typename DenavitHartenberg<T, N>::Transform DenavitHartenberg<T, N>::Forward(const JointVector& q) const
    {
        Transform frame{};
        for (std::size_t i = 0; i < N; ++i)
            frame = frame * LinkTransform(i, q[i]);

        return frame * tool;
    }

    template<typename T, std::size_t N>
    FrameChain<T, N> DenavitHartenberg<T, N>::Frames(const JointVector& q) const
    {
        const math::Vector3<T> zAxis{ T{}, T{}, T{ 1 } };
        const bool standard{ convention == DhConvention::Standard };

        FrameChain<T, N> frames{};
        Transform frame{};
        for (std::size_t i = 0; i < N; ++i)
        {
            if (!standard)
                frame = frame * LinkTransform(i, q[i]);

            frames.jointOrigins[i] = frame.translation;
            frames.jointAxes[i] = frame.rotation * zAxis;
            frames.jointTypes[i] = links[i].type;

            if (standard)
                frame = frame * LinkTransform(i, q[i]);
        }

        frames.tool = frame * tool;
        return frames;
    }

    template<typename T, std::size_t N>
    typename DenavitHartenberg<T, N>::Transform DenavitHartenberg<T, N>::StandardTransform(std::size_t i, T cosTheta, T sinTheta, T d) const
    {
        const T ca{ cosAlpha[i] };
        const T sa{ sinAlpha[i] };
        const T a{ links[i].a };

        return { math::Matrix3<T>{ { cosTheta, -sinTheta * ca, sinTheta * sa }, { sinTheta, cosTheta * ca, -cosTheta * sa }, { T{}, sa, ca } },
            math::Vector3<T>{ a * cosTheta, a * sinTheta, d } };
    }

    template<typename T, std::size_t N>
    typename DenavitHartenberg<T, N>::Transform DenavitHartenberg<T, N>::ModifiedTransform(std::size_t i, T cosTheta, T sinTheta, T d) const
    {
        const T ca{ cosAlpha[i] };
        const T sa{ sinAlpha[i] };

        return { math::Matrix3<T>{ { cosTheta, -sinTheta, T{} }, { sinTheta * ca, cosTheta * ca, -sa }, { sinTheta * sa, cosTheta * sa, ca } },
            math::Vector3<T>{ links[i].a, -sa * d, ca * d } };
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class DenavitHartenberg<float, 1>;
    extern template class DenavitHartenberg<float, 2>;
    extern template class DenavitHartenberg<float, 6>;
#endif
}
