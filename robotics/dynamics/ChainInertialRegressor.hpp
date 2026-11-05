#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/Matrix.hpp"
#include "robotics/dynamics/InertialRegressor.hpp"
#include "robotics/dynamics/JointLink.hpp"
#include "robotics/dynamics/ModifiedRecursiveNewtonEuler.hpp"
#include "robotics/dynamics/SpatialInertia.hpp"
#include <array>
#include <cstddef>
#include <type_traits>

namespace dynamics
{
    template<typename T, std::size_t NumLinks>
    class ChainInertialRegressor
        : public InertialRegressor<T, NumLinks, 10 * NumLinks>
    {
        static_assert(std::is_floating_point_v<T>, "ChainInertialRegressor supports floating-point types");

    public:
        static constexpr std::size_t ParametersPerLink{ 10 };

        using Vector3 = math::Vector<T, 3>;
        using JointVector = math::Vector<T, NumLinks>;
        using Regressor = math::Matrix<T, NumLinks, ParametersPerLink * NumLinks>;
        using ParameterVector = math::Vector<T, ParametersPerLink * NumLinks>;
        using LinkArray = std::array<JointLink<T>, NumLinks>;

        ChainInertialRegressor(const LinkArray& links, const Vector3& gravity);

        OPTIMIZE_FOR_SPEED Regressor Compute(const JointVector& q, const JointVector& qDot, const JointVector& qDotReference, const JointVector& qDDotReference) const override;

        static ParameterVector Parameters(const LinkArray& links);

    private:
        using Columns = std::array<SpatialWrench<T>, ParametersPerLink>;
        using Kinematics = ModifiedRecursiveNewtonEuler<T, NumLinks>;

        static Columns ParameterColumns(const SpatialMotion<T>& motion);
        static Columns LinkColumns(const ReferenceMotion<T, NumLinks>& motion, std::size_t link);
        void Project(Regressor& regressor, Columns columns, const ReferenceMotion<T, NumLinks>& motion, std::size_t link) const;

        const LinkArray& links;
        Vector3 gravity;
    };

    template<typename T, std::size_t NumLinks>
    ChainInertialRegressor<T, NumLinks>::ChainInertialRegressor(const LinkArray& links, const Vector3& gravity)
        : links{ links }
        , gravity{ gravity }
    {}

    template<typename T, std::size_t NumLinks>
    OPTIMIZE_FOR_SPEED typename ChainInertialRegressor<T, NumLinks>::Regressor ChainInertialRegressor<T, NumLinks>::Compute(const JointVector& q, const JointVector& qDot,
        const JointVector& qDotReference, const JointVector& qDDotReference) const
    {
        const auto motion{ Kinematics::ComputeReferenceMotion(links, q, qDot, qDotReference, qDDotReference, gravity) };

        Regressor regressor{};
        for (std::size_t i = 0; i < NumLinks; ++i)
            Project(regressor, LinkColumns(motion, i), motion, i);

        return regressor;
    }

    template<typename T, std::size_t NumLinks>
    typename ChainInertialRegressor<T, NumLinks>::ParameterVector ChainInertialRegressor<T, NumLinks>::Parameters(const LinkArray& links)
    {
        ParameterVector parameters{};
        for (std::size_t i = 0; i < NumLinks; ++i)
        {
            const auto& link{ links[i] };
            const auto aboutJoint{ RigidBodyInertia(link).rot };
            const Vector3 firstMoment{ link.jointToCoM * link.mass };
            const std::array<T, ParametersPerLink> values{ link.mass, firstMoment[0], firstMoment[1], firstMoment[2],
                aboutJoint.at(0, 0), aboutJoint.at(0, 1), aboutJoint.at(0, 2), aboutJoint.at(1, 1), aboutJoint.at(1, 2), aboutJoint.at(2, 2) };
            for (std::size_t k = 0; k < ParametersPerLink; ++k)
                parameters[ParametersPerLink * i + k] = values[k];
        }

        return parameters;
    }

    template<typename T, std::size_t NumLinks>
    typename ChainInertialRegressor<T, NumLinks>::Columns ChainInertialRegressor<T, NumLinks>::ParameterColumns(const SpatialMotion<T>& motion)
    {
        const Vector3& w{ motion.angular };
        const Vector3& u{ motion.linear };
        const std::array<Vector3, 3> axes{ Vector3{ T{ 1 }, T{}, T{} }, Vector3{ T{}, T{ 1 }, T{} }, Vector3{ T{}, T{}, T{ 1 } } };

        Columns columns{};
        columns[0] = SpatialWrench<T>{ u, Vector3{} };
        for (std::size_t k = 0; k < 3; ++k)
            columns[1 + k] = SpatialWrench<T>{ math::CrossProduct(w, axes[k]), math::CrossProduct(axes[k], u) };

        columns[4] = SpatialWrench<T>{ Vector3{}, Vector3{ w[0], T{}, T{} } };
        columns[5] = SpatialWrench<T>{ Vector3{}, Vector3{ w[1], w[0], T{} } };
        columns[6] = SpatialWrench<T>{ Vector3{}, Vector3{ w[2], T{}, w[0] } };
        columns[7] = SpatialWrench<T>{ Vector3{}, Vector3{ T{}, w[1], T{} } };
        columns[8] = SpatialWrench<T>{ Vector3{}, Vector3{ T{}, w[2], w[1] } };
        columns[9] = SpatialWrench<T>{ Vector3{}, Vector3{ T{}, T{}, w[2] } };
        return columns;
    }

    template<typename T, std::size_t NumLinks>
    typename ChainInertialRegressor<T, NumLinks>::Columns ChainInertialRegressor<T, NumLinks>::LinkColumns(const ReferenceMotion<T, NumLinks>& motion, std::size_t link)
    {
        const auto& v{ motion.velocity[link] };
        const auto& vr{ motion.referenceVelocity[link] };
        const auto inertial{ ParameterColumns(motion.acceleration[link]) };
        const auto ofReference{ ParameterColumns(vr) };
        const auto ofCross{ ParameterColumns(MotionCross(v, vr)) };
        const auto ofVelocity{ ParameterColumns(v) };
        const T half{ T{ 0.5f } };

        Columns columns{};
        for (std::size_t k = 0; k < ParametersPerLink; ++k)
        {
            const auto first{ ForceCross(v, ofReference[k]) };
            const auto third{ ForceCross(vr, ofVelocity[k]) };
            columns[k] = SpatialWrench<T>{ inertial[k].force + (first.force - ofCross[k].force + third.force) * half,
                inertial[k].torque + (first.torque - ofCross[k].torque + third.torque) * half };
        }

        return columns;
    }

    template<typename T, std::size_t NumLinks>
    void ChainInertialRegressor<T, NumLinks>::Project(Regressor& regressor, Columns columns, const ReferenceMotion<T, NumLinks>& motion, std::size_t link) const
    {
        for (std::size_t j = link + 1; j > 0; --j)
        {
            const std::size_t joint{ j - 1 };
            const auto subspace{ Kinematics::MotionSubspace(links[joint]) };
            for (std::size_t k = 0; k < ParametersPerLink; ++k)
            {
                regressor.at(joint, ParametersPerLink * link + k) = math::DotProduct(subspace.angular, columns[k].torque) + math::DotProduct(subspace.linear, columns[k].force);
                if (joint > 0)
                    columns[k] = TransformWrenchToParent(columns[k], motion.rotation[joint], motion.offset[joint]);
            }
        }
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class ChainInertialRegressor<float, 2>;
    extern template class ChainInertialRegressor<float, 3>;
#endif
}
