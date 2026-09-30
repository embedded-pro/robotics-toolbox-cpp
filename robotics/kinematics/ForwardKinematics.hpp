#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "robotics/dynamics/RevoluteJointLink.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/Matrix.hpp"
#include <array>

namespace kinematics
{
    template<typename T, std::size_t NumLinks>
    class ForwardKinematics
    {
        static_assert(std::is_floating_point_v<T>,
            "ForwardKinematics only supports floating-point types");
        static_assert(NumLinks > 0, "Number of links must be positive");

    public:
        using Vector3 = math::Vector<T, 3>;
        using Matrix3 = math::SquareMatrix<T, 3>;
        using JointVector = math::Vector<T, NumLinks>;
        using LinkArray = std::array<dynamics::RevoluteJointLink<T>, NumLinks>;
        using PositionArray = std::array<Vector3, NumLinks + 1>;

        explicit ForwardKinematics(const Vector3& toolOffset);

        OPTIMIZE_FOR_SPEED PositionArray Compute(const LinkArray& links,
            const JointVector& q) const;

        const Vector3& ToolOffset() const;

    private:
        Vector3 toolOffset;
    };

    template<typename T, std::size_t NumLinks>
    ForwardKinematics<T, NumLinks>::ForwardKinematics(const Vector3& toolOffset)
        : toolOffset{ toolOffset }
    {}

    template<typename T, std::size_t NumLinks>
    OPTIMIZE_FOR_SPEED
        typename ForwardKinematics<T, NumLinks>::PositionArray
        ForwardKinematics<T, NumLinks>::Compute(const LinkArray& links,
            const JointVector& q) const
    {
        PositionArray positions{};
        positions[0] = links[0].parentToJoint;

        auto R = Matrix3::Identity();

        for (std::size_t i = 0; i < NumLinks; ++i)
        {
            R = R * math::RotationAboutAxis(links[i].jointAxis, q.at(i, 0));

            const Vector3& offsetToNext = (i + 1 < NumLinks) ? links[i + 1].parentToJoint : toolOffset;
            positions[i + 1] = positions[i] + R * offsetToNext;
        }

        return positions;
    }

    template<typename T, std::size_t NumLinks>
    const typename ForwardKinematics<T, NumLinks>::Vector3& ForwardKinematics<T, NumLinks>::ToolOffset() const
    {
        return toolOffset;
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class ForwardKinematics<float, 1>;
    extern template class ForwardKinematics<float, 2>;
    extern template class ForwardKinematics<float, 3>;
#endif
}
