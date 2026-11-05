#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/SE2Transform.hpp"
#include "numerical/math/SE3Transform.hpp"
#include "robotics/kinematics/JacobianProvider.hpp"
#include <cstddef>
#include <type_traits>

namespace kinematics
{
    template<typename T, std::size_t ArmDof>
    class MobileManipulatorKinematics
    {
        static_assert(std::is_floating_point_v<T>, "MobileManipulatorKinematics supports floating-point types");
        static_assert(ArmDof > 0, "Arm must have at least one joint");

    public:
        using Arm = JacobianProvider<T, 6, ArmDof>;
        using JointVector = math::Vector<T, ArmDof>;
        using Jacobian = math::Matrix<T, 6, 2 + ArmDof>;
        using Constraint = math::Matrix<T, 3, 2>;

        MobileManipulatorKinematics(const Arm& arm, const math::SE3Transform<T>& mount);

        OPTIMIZE_FOR_SPEED Jacobian Compute(const math::SE2Transform<T>& base, const JointVector& q) const;
        math::SE3Transform<T> ToolPose(const math::SE2Transform<T>& base, const JointVector& q) const;

        static Constraint BaseConstraint(T phi);

    private:
        const Arm& arm;
        math::SE3Transform<T> mount;
    };

    template<typename T, std::size_t ArmDof>
    MobileManipulatorKinematics<T, ArmDof>::MobileManipulatorKinematics(const Arm& arm, const math::SE3Transform<T>& mount)
        : arm{ arm }
        , mount{ mount }
    {}

    template<typename T, std::size_t ArmDof>
    OPTIMIZE_FOR_SPEED auto MobileManipulatorKinematics<T, ArmDof>::Compute(const math::SE2Transform<T>& base, const JointVector& q) const -> Jacobian
    {
        const math::Vector3<T> up{ T{}, T{}, T{ 1 } };
        const math::Matrix3<T> armRotation{ base.ToSE3().rotation * mount.rotation };
        const auto armJacobian{ arm.Jacobian(q) };
        const auto tool{ ToolPose(base, q).translation };
        const math::Vector3<T> lever{ tool[0] - base.translation[0], tool[1] - base.translation[1], tool[2] };

        Jacobian jacobian{};
        jacobian.SetBlock(math::Vector3<T>{ base.cosine, base.sine, T{} }, 0, 0);
        jacobian.SetBlock(math::CrossProduct(up, lever), 0, 1);
        jacobian.SetBlock(up, 3, 1);
        jacobian.SetBlock(armRotation * armJacobian.template GetBlock<3, ArmDof>(0, 0), 0, 2);
        jacobian.SetBlock(armRotation * armJacobian.template GetBlock<3, ArmDof>(3, 0), 3, 2);
        return jacobian;
    }

    template<typename T, std::size_t ArmDof>
    math::SE3Transform<T> MobileManipulatorKinematics<T, ArmDof>::ToolPose(const math::SE2Transform<T>& base, const JointVector& q) const
    {
        return base.ToSE3() * mount * arm.ToolPose(q);
    }

    template<typename T, std::size_t ArmDof>
    auto MobileManipulatorKinematics<T, ArmDof>::BaseConstraint(T phi) -> Constraint
    {
        return Constraint{ { math::Cos(phi), T{} }, { math::Sin(phi), T{} }, { T{}, T{ 1 } } };
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class MobileManipulatorKinematics<float, 3>;
    extern template class MobileManipulatorKinematics<float, 6>;
#endif
}
