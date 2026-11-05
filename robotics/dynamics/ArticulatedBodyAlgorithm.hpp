#pragma once

#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/Matrix.hpp"
#include "robotics/dynamics/JointLink.hpp"
#include "robotics/dynamics/SpatialInertia.hpp"
#include <array>
#include <cassert>

namespace dynamics
{
    template<typename T, std::size_t NumLinks>
    class ArticulatedBodyAlgorithm
    {
        static_assert(std::is_floating_point_v<T>,
            "ArticulatedBodyAlgorithm only supports floating-point types");
        static_assert(NumLinks > 0, "Number of links must be positive");

    public:
        using Vector3 = math::Vector<T, 3>;
        using Matrix3 = math::SquareMatrix<T, 3>;
        using JointVector = math::Vector<T, NumLinks>;
        using LinkArray = std::array<JointLink<T>, NumLinks>;

        ArticulatedBodyAlgorithm() = default;

        OPTIMIZE_FOR_SPEED JointVector ForwardDynamics(const LinkArray& links,
            const JointVector& q, const JointVector& qDot, const JointVector& tau,
            const Vector3& gravity) const;

    private:
        struct LinkKinematics
        {
            Vector3 omega;
            Vector3 linVelJ;
            Matrix3 R;
            Vector3 offset;
            Vector3 cJ;
            Vector3 cJLin;
        };

        using SpatialInertia = dynamics::SpatialInertia<T>;
        using BiasWrench = SpatialWrench<T>;

        struct JointProjection
        {
            T D;
            T u;
            Vector3 Ua;
            Vector3 UaLin;
        };

        static std::array<LinkKinematics, NumLinks> ComputeForwardKinematics(
            const LinkArray& links, const JointVector& q, const JointVector& qDot);

        static void PropagateKinematicsFromParent(
            LinkKinematics& current, const LinkKinematics& parent,
            const JointLink<T>& link, T qDot_i);

        static BiasWrench ComputeBiasWrench(
            const JointLink<T>& link,
            const LinkKinematics& kin);

        static JointProjection ComputeJointProjection(
            const JointLink<T>& link, const SpatialInertia& Ia,
            const BiasWrench& pA, T tau);

        static SpatialInertia ArticulatedBodyDowndate(
            const SpatialInertia& Ia, const JointProjection& proj);

        static BiasWrench ComputeArticulatedBiasWrench(
            const BiasWrench& pA, const SpatialInertia& Ia,
            const LinkKinematics& kin, const JointProjection& proj);

        static void AccumulateChildContribution(
            const LinkArray& links, std::size_t childIndex,
            std::array<SpatialInertia, NumLinks>& Ia,
            std::array<BiasWrench, NumLinks>& pA,
            const std::array<LinkKinematics, NumLinks>& kin,
            const std::array<JointProjection, NumLinks>& proj);

        static void ComputeArticulatedBodyInertias(
            const LinkArray& links, const JointVector& tau,
            const std::array<LinkKinematics, NumLinks>& kin,
            std::array<SpatialInertia, NumLinks>& Ia,
            std::array<BiasWrench, NumLinks>& pA,
            std::array<JointProjection, NumLinks>& proj);

        static T ComputeJointAcceleration(
            const JointProjection& proj,
            const Vector3& aHatAng, const Vector3& aHatLin);

        static JointVector ComputeJointAccelerations(
            const LinkArray& links, const Vector3& gravity,
            const std::array<LinkKinematics, NumLinks>& kin,
            const std::array<JointProjection, NumLinks>& proj);
    };

    template<typename T, std::size_t NumLinks>
    void ArticulatedBodyAlgorithm<T, NumLinks>::PropagateKinematicsFromParent(
        LinkKinematics& current, const LinkKinematics& parent,
        const JointLink<T>& link, T qDot_i)
    {
        const auto Rt = current.R.Transpose();
        const Vector3 angularRate{ link.AngularAxis() * qDot_i };
        const Vector3 linearRate{ link.LinearAxis() * qDot_i };

        current.omega = Rt * parent.omega + angularRate;
        current.linVelJ = Rt * (parent.linVelJ + math::CrossProduct(parent.omega, current.offset)) + linearRate;

        current.cJ = math::CrossProduct(current.omega, angularRate);
        current.cJLin = math::CrossProduct(current.linVelJ, angularRate) + math::CrossProduct(current.omega, linearRate);
    }

    template<typename T, std::size_t NumLinks>
    std::array<typename ArticulatedBodyAlgorithm<T, NumLinks>::LinkKinematics, NumLinks>
    ArticulatedBodyAlgorithm<T, NumLinks>::ComputeForwardKinematics(
        const LinkArray& links, const JointVector& q, const JointVector& qDot)
    {
        std::array<LinkKinematics, NumLinks> kin{};
        const LinkKinematics base{};

        for (std::size_t i = 0; i < NumLinks; ++i)
        {
            assert(HasUnitJointAxis(links[i]));
            const auto transform{ links[i].JointTransform(q.at(i, 0)) };
            kin[i].R = transform.rotation;
            kin[i].offset = transform.translation;

            PropagateKinematicsFromParent(kin[i], i == 0 ? base : kin[i - 1], links[i], qDot.at(i, 0));
        }

        return kin;
    }

    template<typename T, std::size_t NumLinks>
    typename ArticulatedBodyAlgorithm<T, NumLinks>::BiasWrench
    ArticulatedBodyAlgorithm<T, NumLinks>::ComputeBiasWrench(
        const JointLink<T>& link,
        const LinkKinematics& kin)
    {
        auto rCoM = link.jointToCoM;
        auto velCoM = kin.linVelJ + math::CrossProduct(kin.omega, rCoM);
        auto biasForce = math::CrossProduct(kin.omega, velCoM) * link.mass;
        auto Iw = link.inertia * kin.omega;
        auto biasTorque = math::CrossProduct(kin.omega, Iw);

        return BiasWrench{
            biasForce,
            biasTorque + math::CrossProduct(rCoM, biasForce)
        };
    }

    template<typename T, std::size_t NumLinks>
    typename ArticulatedBodyAlgorithm<T, NumLinks>::JointProjection
    ArticulatedBodyAlgorithm<T, NumLinks>::ComputeJointProjection(
        const JointLink<T>& link, const SpatialInertia& Ia,
        const BiasWrench& pA, T tau)
    {
        const Vector3 angular{ link.AngularAxis() };
        const Vector3 linear{ link.LinearAxis() };

        JointProjection proj{};
        proj.Ua = Ia.rot * angular + Ia.cross * linear;
        proj.UaLin = Ia.cross.Transpose() * angular + Ia.lin * linear;

        proj.D = math::DotProduct(angular, proj.Ua) + math::DotProduct(linear, proj.UaLin) + link.armature;
        really_assert(proj.D > T(0));

        proj.u = tau - math::DotProduct(angular, pA.torque) - math::DotProduct(linear, pA.force);

        return proj;
    }

    template<typename T, std::size_t NumLinks>
    typename ArticulatedBodyAlgorithm<T, NumLinks>::SpatialInertia
    ArticulatedBodyAlgorithm<T, NumLinks>::ArticulatedBodyDowndate(
        const SpatialInertia& Ia, const JointProjection& proj)
    {
        T invD = T(1) / proj.D;

        return SpatialInertia{
            Ia.rot - math::OuterProduct(proj.Ua, proj.Ua) * invD,
            Ia.cross - math::OuterProduct(proj.Ua, proj.UaLin) * invD,
            Ia.lin - math::OuterProduct(proj.UaLin, proj.UaLin) * invD
        };
    }

    template<typename T, std::size_t NumLinks>
    typename ArticulatedBodyAlgorithm<T, NumLinks>::BiasWrench
    ArticulatedBodyAlgorithm<T, NumLinks>::ComputeArticulatedBiasWrench(
        const BiasWrench& pA, const SpatialInertia& Ia,
        const LinkKinematics& kin, const JointProjection& proj)
    {
        auto uScaled = proj.u / proj.D;

        return BiasWrench{
            pA.force + Ia.cross.Transpose() * kin.cJ + Ia.lin * kin.cJLin + proj.UaLin * uScaled,
            pA.torque + Ia.rot * kin.cJ + Ia.cross * kin.cJLin + proj.Ua * uScaled
        };
    }

    template<typename T, std::size_t NumLinks>
    T ArticulatedBodyAlgorithm<T, NumLinks>::ComputeJointAcceleration(
        const JointProjection& proj,
        const Vector3& aPrimeAng, const Vector3& aPrimeLin)
    {
        return (proj.u - math::DotProduct(proj.Ua, aPrimeAng) - math::DotProduct(proj.UaLin, aPrimeLin)) / proj.D;
    }

    template<typename T, std::size_t NumLinks>
    void ArticulatedBodyAlgorithm<T, NumLinks>::AccumulateChildContribution(
        const LinkArray& links, std::size_t childIndex,
        std::array<SpatialInertia, NumLinks>& Ia,
        std::array<BiasWrench, NumLinks>& pA,
        const std::array<LinkKinematics, NumLinks>& kin,
        const std::array<JointProjection, NumLinks>& proj)
    {
        auto parentIndex = childIndex - 1;

        auto IaA = ArticulatedBodyDowndate(Ia[childIndex], proj[childIndex]);
        auto pAA = ComputeArticulatedBiasWrench(pA[childIndex], IaA, kin[childIndex], proj[childIndex]);

        auto IaParent = TransformInertiaToParent(IaA, kin[childIndex].R, kin[childIndex].offset);
        auto pAParent = TransformWrenchToParent(pAA, kin[childIndex].R, kin[childIndex].offset);

        Ia[parentIndex] += IaParent;
        pA[parentIndex].force = pA[parentIndex].force + pAParent.force;
        pA[parentIndex].torque = pA[parentIndex].torque + pAParent.torque;
    }

    template<typename T, std::size_t NumLinks>
    void ArticulatedBodyAlgorithm<T, NumLinks>::ComputeArticulatedBodyInertias(
        const LinkArray& links, const JointVector& tau,
        const std::array<LinkKinematics, NumLinks>& kin,
        std::array<SpatialInertia, NumLinks>& Ia,
        std::array<BiasWrench, NumLinks>& pA,
        std::array<JointProjection, NumLinks>& proj)
    {
        for (std::size_t i = 0; i < NumLinks; ++i)
        {
            Ia[i] = RigidBodyInertia(links[i]);
            pA[i] = ComputeBiasWrench(links[i], kin[i]);
        }

        for (std::size_t j = NumLinks; j > 0; --j)
        {
            std::size_t i = j - 1;
            proj[i] = ComputeJointProjection(links[i], Ia[i], pA[i], tau.at(i, 0));

            if (i > 0)
                AccumulateChildContribution(links, i, Ia, pA, kin, proj);
        }
    }

    template<typename T, std::size_t NumLinks>
    typename ArticulatedBodyAlgorithm<T, NumLinks>::JointVector
    ArticulatedBodyAlgorithm<T, NumLinks>::ComputeJointAccelerations(
        const LinkArray& links, const Vector3& gravity,
        const std::array<LinkKinematics, NumLinks>& kin,
        const std::array<JointProjection, NumLinks>& proj)
    {
        JointVector qDDot;
        std::array<Vector3, NumLinks> angAcc;
        std::array<Vector3, NumLinks> linAcc;

        for (std::size_t i = 0; i < NumLinks; ++i)
        {
            const auto Rt = kin[i].R.Transpose();
            const Vector3 parentAng{ i == 0 ? Vector3{} : angAcc[i - 1] };
            const Vector3 parentLin{ i == 0 ? Vector3{ gravity * T(-1) } : linAcc[i - 1] };

            const Vector3 aHatAng{ Rt * parentAng + kin[i].cJ };
            const Vector3 aHatLin{ Rt * (parentLin + math::CrossProduct(parentAng, kin[i].offset)) + kin[i].cJLin };

            qDDot.at(i, 0) = ComputeJointAcceleration(proj[i], aHatAng, aHatLin);

            angAcc[i] = aHatAng + links[i].AngularAxis() * qDDot.at(i, 0);
            linAcc[i] = aHatLin + links[i].LinearAxis() * qDDot.at(i, 0);
        }

        return qDDot;
    }

    template<typename T, std::size_t NumLinks>
    OPTIMIZE_FOR_SPEED
        typename ArticulatedBodyAlgorithm<T, NumLinks>::JointVector
        ArticulatedBodyAlgorithm<T, NumLinks>::ForwardDynamics(const LinkArray& links,
            const JointVector& q, const JointVector& qDot, const JointVector& tau,
            const Vector3& gravity) const
    {
        auto kin = ComputeForwardKinematics(links, q, qDot);

        std::array<SpatialInertia, NumLinks> Ia;
        std::array<BiasWrench, NumLinks> pA;
        std::array<JointProjection, NumLinks> proj;
        ComputeArticulatedBodyInertias(links, tau, kin, Ia, pA, proj);

        return ComputeJointAccelerations(links, gravity, kin, proj);
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class ArticulatedBodyAlgorithm<float, 1>;
    extern template class ArticulatedBodyAlgorithm<float, 2>;
    extern template class ArticulatedBodyAlgorithm<float, 3>;
#endif
}
