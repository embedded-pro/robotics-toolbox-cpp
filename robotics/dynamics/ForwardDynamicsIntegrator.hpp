#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/solvers/OdeSystem.hpp"
#include "numerical/solvers/RungeKuttaIntegrators.hpp"
#include "numerical/solvers/SecondOrderSystem.hpp"
#include "numerical/solvers/SymplecticIntegrators.hpp"
#include "robotics/dynamics/ArticulatedBodyAlgorithm.hpp"
#include "robotics/dynamics/RevoluteJointLink.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace dynamics
{
    enum class IntegrationMethod : std::uint8_t
    {
        SemiImplicitEuler,
        RungeKutta4
    };

    template<typename T, std::size_t NumLinks>
    struct ChainState
    {
        static_assert(std::is_floating_point_v<T>, "ChainState supports floating-point types");

        math::Vector<T, NumLinks> q{};
        math::Vector<T, NumLinks> qDot{};
        math::Vector<T, NumLinks> qDDot{};
    };

    template<typename T, std::size_t NumLinks>
    class ChainAccelerationSystem
        : public solvers::SecondOrderSystem<T, NumLinks, NumLinks>
        , public solvers::OdeSystem<T, 2 * NumLinks, NumLinks>
    {
        static_assert(std::is_floating_point_v<T>, "ChainAccelerationSystem supports floating-point types");

    public:
        using Vector3 = math::Vector<T, 3>;
        using JointVector = math::Vector<T, NumLinks>;
        using StateVector = math::Vector<T, 2 * NumLinks>;
        using LinkArray = std::array<RevoluteJointLink<T>, NumLinks>;

        ChainAccelerationSystem(const LinkArray& links, const Vector3& gravity);

        JointVector Acceleration(const JointVector& q, const JointVector& qDot, const JointVector& tau) override;
        StateVector Derivative(const StateVector& x, const JointVector& tau, T t) override;

    private:
        const LinkArray& links;
        Vector3 gravity;
        [[no_unique_address]] ArticulatedBodyAlgorithm<T, NumLinks> aba;
    };

    template<typename T, std::size_t NumLinks, IntegrationMethod Method>
    class ForwardDynamicsIntegrator
    {
        static_assert(std::is_floating_point_v<T>, "ForwardDynamicsIntegrator supports floating-point types");

    public:
        using Vector3 = math::Vector<T, 3>;
        using JointVector = math::Vector<T, NumLinks>;
        using LinkArray = std::array<RevoluteJointLink<T>, NumLinks>;
        using State = ChainState<T, NumLinks>;

        ForwardDynamicsIntegrator(const LinkArray& links, const Vector3& gravity, T dt);

        OPTIMIZE_FOR_SPEED State Step(const State& state, const JointVector& tau);

    private:
        using Integrator = std::conditional_t<Method == IntegrationMethod::SemiImplicitEuler,
            solvers::SemiImplicitEuler<T, NumLinks, NumLinks>,
            solvers::RungeKutta4<T, 2 * NumLinks, NumLinks>>;

        ChainAccelerationSystem<T, NumLinks> system;
        Integrator integrator;
    };

    template<typename T, std::size_t NumLinks>
    ChainAccelerationSystem<T, NumLinks>::ChainAccelerationSystem(const LinkArray& links, const Vector3& gravity)
        : links{ links }
        , gravity{ gravity }
    {}

    template<typename T, std::size_t NumLinks>
    typename ChainAccelerationSystem<T, NumLinks>::JointVector ChainAccelerationSystem<T, NumLinks>::Acceleration(const JointVector& q, const JointVector& qDot, const JointVector& tau)
    {
        return aba.ForwardDynamics(links, q, qDot, tau, gravity);
    }

    template<typename T, std::size_t NumLinks>
    typename ChainAccelerationSystem<T, NumLinks>::StateVector ChainAccelerationSystem<T, NumLinks>::Derivative(const StateVector& x, const JointVector& tau, T)
    {
        const auto qDot{ x.template GetBlock<NumLinks, 1>(NumLinks, 0) };

        StateVector derivative{};
        derivative.SetBlock(qDot, 0, 0);
        derivative.SetBlock(Acceleration(x.template GetBlock<NumLinks, 1>(0, 0), qDot, tau), NumLinks, 0);
        return derivative;
    }

    template<typename T, std::size_t NumLinks, IntegrationMethod Method>
    ForwardDynamicsIntegrator<T, NumLinks, Method>::ForwardDynamicsIntegrator(const LinkArray& links, const Vector3& gravity, T dt)
        : system{ links, gravity }
        , integrator{ system, dt }
    {}

    template<typename T, std::size_t NumLinks, IntegrationMethod Method>
    OPTIMIZE_FOR_SPEED typename ForwardDynamicsIntegrator<T, NumLinks, Method>::State ForwardDynamicsIntegrator<T, NumLinks, Method>::Step(const State& state, const JointVector& tau)
    {
        const auto acceleration{ system.Acceleration(state.q, state.qDot, tau) };

        if constexpr (Method == IntegrationMethod::SemiImplicitEuler)
        {
            const auto next{ integrator.Step({ state.q, state.qDot }, tau) };
            return { next.q, next.qDot, acceleration };
        }
        else
        {
            math::Vector<T, 2 * NumLinks> x{};
            x.SetBlock(state.q, 0, 0);
            x.SetBlock(state.qDot, NumLinks, 0);
            const auto next{ integrator.Step(x, tau, T{}) };
            return { next.template GetBlock<NumLinks, 1>(0, 0), next.template GetBlock<NumLinks, 1>(NumLinks, 0), acceleration };
        }
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class ChainAccelerationSystem<float, 1>;
    extern template class ChainAccelerationSystem<float, 2>;
    extern template class ForwardDynamicsIntegrator<float, 1, IntegrationMethod::RungeKutta4>;
    extern template class ForwardDynamicsIntegrator<float, 2, IntegrationMethod::SemiImplicitEuler>;
#endif
}
