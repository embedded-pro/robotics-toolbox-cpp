#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/Matrix.hpp"
#include "robotics/dynamics/JointLink.hpp"
#include <array>
#include <cassert>
#include <type_traits>

namespace dynamics
{
    template<typename T>
    struct ToolWrench
    {
        static_assert(std::is_floating_point_v<T>, "ToolWrench supports floating-point types");

        math::Vector<T, 3> force{};
        math::Vector<T, 3> moment{};
        math::Vector<T, 3> toolOffset{};
    };

    template<typename T, std::size_t NumLinks>
    class RecursiveNewtonEuler
    {
        static_assert(std::is_floating_point_v<T>,
            "RecursiveNewtonEuler only supports floating-point types");
        static_assert(NumLinks > 0, "Number of links must be positive");

    public:
        using Vector3 = math::Vector<T, 3>;
        using Matrix3 = math::SquareMatrix<T, 3>;
        using JointVector = math::Vector<T, NumLinks>;
        using LinkArray = std::array<JointLink<T>, NumLinks>;

        RecursiveNewtonEuler() = default;

        OPTIMIZE_FOR_SPEED JointVector InverseDynamics(const LinkArray& links,
            const JointVector& q, const JointVector& qDot, const JointVector& qDDot,
            const Vector3& gravity) const;

        OPTIMIZE_FOR_SPEED JointVector InverseDynamics(const LinkArray& links,
            const JointVector& q, const JointVector& qDot, const JointVector& qDDot,
            const Vector3& gravity, const ToolWrench<T>& wrench) const;

    private:
        struct LinkState
        {
            Vector3 omega;
            Vector3 omegaDot;
            Vector3 linAccJoint;
            Matrix3 R;
            Vector3 offset;
        };

        static void PropagateStateFromParent(
            LinkState& state, const LinkState& parent,
            const JointLink<T>& link, T qDot_i, T qDDot_i);

        static std::array<LinkState, NumLinks> ComputeForwardPass(
            const LinkArray& links, const JointVector& q,
            const JointVector& qDot, const JointVector& qDDot,
            const Vector3& gravity);

        static void ComputeLinkWrench(
            Vector3& force, Vector3& torque,
            const JointLink<T>& link, const LinkState& state);

        static void AccumulateChildWrench(
            Vector3& parentForce, Vector3& parentTorque,
            const Vector3& childForce, const Vector3& childTorque,
            const Matrix3& childR, const Vector3& rToChild);

        static JointVector ComputeBackwardPass(
            const LinkArray& links, const std::array<LinkState, NumLinks>& states,
            const JointVector& qDDot, const Vector3& tipForce, const Vector3& tipTorque);
    };

    template<typename T, std::size_t NumLinks>
    void RecursiveNewtonEuler<T, NumLinks>::PropagateStateFromParent(
        LinkState& state, const LinkState& parent,
        const JointLink<T>& link, T qDot_i, T qDDot_i)
    {
        const auto Rt = state.R.Transpose();
        const Vector3 angularRate{ link.AngularAxis() * qDot_i };
        const Vector3 linearRate{ link.LinearAxis() * qDot_i };

        state.omega = Rt * parent.omega + angularRate;
        state.omegaDot = Rt * parent.omegaDot + math::CrossProduct(state.omega, angularRate) + link.AngularAxis() * qDDot_i;

        const Vector3 accAtJoint{ parent.linAccJoint + math::CrossProduct(parent.omegaDot, state.offset) + math::CrossProduct(parent.omega, math::CrossProduct(parent.omega, state.offset)) };
        state.linAccJoint = Rt * accAtJoint + math::CrossProduct(state.omega, linearRate) * T(2) + link.LinearAxis() * qDDot_i;
    }

    template<typename T, std::size_t NumLinks>
    std::array<typename RecursiveNewtonEuler<T, NumLinks>::LinkState, NumLinks>
    RecursiveNewtonEuler<T, NumLinks>::ComputeForwardPass(
        const LinkArray& links, const JointVector& q,
        const JointVector& qDot, const JointVector& qDDot,
        const Vector3& gravity)
    {
        std::array<LinkState, NumLinks> states{};
        const LinkState base{ Vector3{}, Vector3{}, gravity * T(-1), Matrix3::Identity(), Vector3{} };

        for (std::size_t i = 0; i < NumLinks; ++i)
        {
            assert(HasUnitJointAxis(links[i]));
            const auto transform{ links[i].JointTransform(q.at(i, 0)) };
            states[i].R = transform.rotation;
            states[i].offset = transform.translation;

            PropagateStateFromParent(states[i], i == 0 ? base : states[i - 1], links[i], qDot.at(i, 0), qDDot.at(i, 0));
        }

        return states;
    }

    template<typename T, std::size_t NumLinks>
    void RecursiveNewtonEuler<T, NumLinks>::ComputeLinkWrench(
        Vector3& force, Vector3& torque,
        const JointLink<T>& link, const LinkState& state)
    {
        const Vector3 rCoM{ link.jointToCoM };
        const Vector3 linAccCoM{ state.linAccJoint + math::CrossProduct(state.omegaDot, rCoM) + math::CrossProduct(state.omega, math::CrossProduct(state.omega, rCoM)) };
        force = linAccCoM * link.mass;

        auto Iw = link.inertia * state.omega;
        torque = link.inertia * state.omegaDot + math::CrossProduct(state.omega, Iw);

        torque = torque + math::CrossProduct(rCoM, force);
    }

    template<typename T, std::size_t NumLinks>
    void RecursiveNewtonEuler<T, NumLinks>::AccumulateChildWrench(
        Vector3& parentForce, Vector3& parentTorque,
        const Vector3& childForce, const Vector3& childTorque,
        const Matrix3& childR, const Vector3& rToChild)
    {
        auto childForceInParent = childR * childForce;
        auto childTorqueInParent = childR * childTorque;

        parentForce = parentForce + childForceInParent;
        parentTorque = parentTorque + childTorqueInParent + math::CrossProduct(rToChild, childForceInParent);
    }

    template<typename T, std::size_t NumLinks>
    typename RecursiveNewtonEuler<T, NumLinks>::JointVector
    RecursiveNewtonEuler<T, NumLinks>::ComputeBackwardPass(
        const LinkArray& links, const std::array<LinkState, NumLinks>& states,
        const JointVector& qDDot, const Vector3& tipForce, const Vector3& tipTorque)
    {
        std::array<Vector3, NumLinks> force;
        std::array<Vector3, NumLinks> torque;

        for (std::size_t j = NumLinks; j > 0; --j)
        {
            std::size_t i = j - 1;

            ComputeLinkWrench(force[i], torque[i], links[i], states[i]);

            if (i + 1 < NumLinks)
                AccumulateChildWrench(force[i], torque[i], force[i + 1], torque[i + 1],
                    states[i + 1].R, states[i + 1].offset);
            else
            {
                force[i] = force[i] - tipForce;
                torque[i] = torque[i] - tipTorque;
            }
        }

        JointVector tau{};
        for (std::size_t i = 0; i < NumLinks; ++i)
            tau.at(i, 0) = math::DotProduct(links[i].AngularAxis(), torque[i]) + math::DotProduct(links[i].LinearAxis(), force[i]) + links[i].armature * qDDot.at(i, 0);

        return tau;
    }

    template<typename T, std::size_t NumLinks>
    OPTIMIZE_FOR_SPEED
        typename RecursiveNewtonEuler<T, NumLinks>::JointVector
        RecursiveNewtonEuler<T, NumLinks>::InverseDynamics(const LinkArray& links,
            const JointVector& q, const JointVector& qDot, const JointVector& qDDot,
            const Vector3& gravity) const
    {
        auto states = ComputeForwardPass(links, q, qDot, qDDot, gravity);
        return ComputeBackwardPass(links, states, qDDot, Vector3{}, Vector3{});
    }

    template<typename T, std::size_t NumLinks>
    OPTIMIZE_FOR_SPEED
        typename RecursiveNewtonEuler<T, NumLinks>::JointVector
        RecursiveNewtonEuler<T, NumLinks>::InverseDynamics(const LinkArray& links,
            const JointVector& q, const JointVector& qDot, const JointVector& qDDot,
            const Vector3& gravity, const ToolWrench<T>& wrench) const
    {
        auto states = ComputeForwardPass(links, q, qDot, qDDot, gravity);

        Matrix3 baseFromLast{ Matrix3::Identity() };
        for (const auto& state : states)
            baseFromLast = baseFromLast * state.R;

        const Vector3 tipForce{ baseFromLast.Transpose() * wrench.force };
        const Vector3 tipTorque{ baseFromLast.Transpose() * wrench.moment + math::CrossProduct(wrench.toolOffset, tipForce) };
        return ComputeBackwardPass(links, states, qDDot, tipForce, tipTorque);
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class RecursiveNewtonEuler<float, 1>;
    extern template class RecursiveNewtonEuler<float, 2>;
    extern template class RecursiveNewtonEuler<float, 3>;
#endif
}
