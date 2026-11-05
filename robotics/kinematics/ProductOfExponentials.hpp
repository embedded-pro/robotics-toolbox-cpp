#pragma once

#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/SE3Transform.hpp"
#include "robotics/dynamics/JointLink.hpp"
#include "robotics/kinematics/ChainPoseKinematics.hpp"
#include <array>
#include <cstddef>
#include <type_traits>

namespace kinematics
{
    template<typename T, std::size_t N>
    class ProductOfExponentials
    {
        static_assert(std::is_floating_point_v<T>, "ProductOfExponentials supports floating-point types");
        static_assert(N > 0, "Number of joints must be positive");

    public:
        using JointVector = math::Vector<T, N>;
        using Screw = math::Vector6<T>;
        using Vector3 = math::Vector3<T>;
        using Transform = math::SE3Transform<T>;
        using Jacobian = math::Matrix<T, 6, N>;

        ProductOfExponentials(const std::array<Screw, N>& screws, const Transform& home);

        OPTIMIZE_FOR_SPEED Transform Compute(const JointVector& q) const;
        Jacobian SpaceJacobian(const JointVector& q) const;
        FrameChain<T, N> Frames(const JointVector& q) const;
        static Jacobian SpaceToGeometric(const Jacobian& space, const Vector3& toolPoint);

    private:
        static constexpr T pitchTolerance{ static_cast<T>(1e-5) };

        static Vector3 Linear(const Screw& screw);
        static Vector3 Angular(const Screw& screw);
        static void PlaceJoint(FrameChain<T, N>& frames, std::size_t joint, const Screw& current, const Vector3& framePosition);

        std::array<Screw, N> screws;
        Transform home;
    };

    template<typename T, std::size_t N>
    ProductOfExponentials<T, N>::ProductOfExponentials(const std::array<Screw, N>& screws, const Transform& home)
        : screws{ screws }
        , home{ home }
    {
        for (const auto& screw : screws)
            really_assert(math::Abs(math::DotProduct(Linear(screw), Angular(screw))) < pitchTolerance);
    }

    template<typename T, std::size_t N>
    OPTIMIZE_FOR_SPEED typename ProductOfExponentials<T, N>::Transform ProductOfExponentials<T, N>::Compute(const JointVector& q) const
    {
        Transform product{};
        for (std::size_t i = 0; i < N; ++i)
            product = product * Transform::Exp(screws[i], q[i]);

        return product * home;
    }

    template<typename T, std::size_t N>
    typename ProductOfExponentials<T, N>::Jacobian ProductOfExponentials<T, N>::SpaceJacobian(const JointVector& q) const
    {
        Jacobian jacobian{};
        Transform product{};
        for (std::size_t i = 0; i < N; ++i)
        {
            jacobian.SetBlock(product.TransformTwist(screws[i]), 0, i);
            product = product * Transform::Exp(screws[i], q[i]);
        }

        return jacobian;
    }

    template<typename T, std::size_t N>
    FrameChain<T, N> ProductOfExponentials<T, N>::Frames(const JointVector& q) const
    {
        FrameChain<T, N> frames{};
        Transform product{};
        for (std::size_t i = 0; i < N; ++i)
        {
            PlaceJoint(frames, i, product.TransformTwist(screws[i]), product.translation);
            product = product * Transform::Exp(screws[i], q[i]);
        }

        frames.tool = product * home;
        return frames;
    }

    template<typename T, std::size_t N>
    typename ProductOfExponentials<T, N>::Jacobian ProductOfExponentials<T, N>::SpaceToGeometric(const Jacobian& space, const Vector3& toolPoint)
    {
        Jacobian geometric{ space };
        for (std::size_t i = 0; i < N; ++i)
        {
            const Screw column{ space.GetColumn(i) };
            geometric.SetBlock(Vector3{ Linear(column) + math::CrossProduct(Angular(column), toolPoint) }, 0, i);
        }

        return geometric;
    }

    template<typename T, std::size_t N>
    typename ProductOfExponentials<T, N>::Vector3 ProductOfExponentials<T, N>::Linear(const Screw& screw)
    {
        return screw.template GetBlock<3, 1>(0, 0);
    }

    template<typename T, std::size_t N>
    typename ProductOfExponentials<T, N>::Vector3 ProductOfExponentials<T, N>::Angular(const Screw& screw)
    {
        return screw.template GetBlock<3, 1>(3, 0);
    }

    template<typename T, std::size_t N>
    void ProductOfExponentials<T, N>::PlaceJoint(FrameChain<T, N>& frames, std::size_t joint, const Screw& current, const Vector3& framePosition)
    {
        const Vector3 omega{ Angular(current) };
        const Vector3 v{ Linear(current) };
        const T omegaSquared{ math::DotProduct(omega, omega) };

        if (omegaSquared > pitchTolerance)
        {
            frames.jointAxes[joint] = omega * (T{ 1 } / math::Sqrt(omegaSquared));
            frames.jointOrigins[joint] = math::CrossProduct(omega, v) * (T{ 1 } / omegaSquared);
            frames.jointTypes[joint] = dynamics::JointType::Revolute;
        }
        else
        {
            frames.jointAxes[joint] = v * (T{ 1 } / math::VectorNorm(v));
            frames.jointOrigins[joint] = framePosition;
            frames.jointTypes[joint] = dynamics::JointType::Prismatic;
        }
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class ProductOfExponentials<float, 1>;
    extern template class ProductOfExponentials<float, 2>;
    extern template class ProductOfExponentials<float, 6>;
#endif
}
