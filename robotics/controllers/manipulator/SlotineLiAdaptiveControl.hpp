#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include "robotics/dynamics/InertialRegressor.hpp"
#include <cstddef>
#include <type_traits>

namespace controllers
{
    template<typename T, std::size_t Dof, std::size_t NumParams>
    class SlotineLiAdaptiveControl
    {
        static_assert(std::is_floating_point_v<T>, "SlotineLiAdaptiveControl supports floating-point types");
        static_assert(Dof > 0 && NumParams > 0, "Dimensions must be positive");

    public:
        using StateVector = math::Vector<T, Dof>;
        using GainMatrix = math::SquareMatrix<T, Dof>;
        using ParameterVector = math::Vector<T, NumParams>;
        using AdaptationMatrix = math::SquareMatrix<T, NumParams>;
        using Regressor = dynamics::InertialRegressor<T, Dof, NumParams>;

        SlotineLiAdaptiveControl(const Regressor& regressor, const GainMatrix& lambda, const GainMatrix& kd, const AdaptationMatrix& gamma, const ParameterVector& initialEstimate, T dt);

        OPTIMIZE_FOR_SPEED StateVector ComputeTorque(const StateVector& q, const StateVector& qDot, const StateVector& qd, const StateVector& qdDot, const StateVector& qdDdot);
        const ParameterVector& ParameterEstimate() const;
        void Reset(const ParameterVector& initialEstimate);

    private:
        const Regressor& regressor;
        GainMatrix lambda;
        GainMatrix kd;
        AdaptationMatrix gamma;
        ParameterVector estimate;
        T dt;
    };

    template<typename T, std::size_t Dof, std::size_t NumParams>
    SlotineLiAdaptiveControl<T, Dof, NumParams>::SlotineLiAdaptiveControl(const Regressor& regressor, const GainMatrix& lambda, const GainMatrix& kd, const AdaptationMatrix& gamma,
        const ParameterVector& initialEstimate, T dt)
        : regressor{ regressor }
        , lambda{ lambda }
        , kd{ kd }
        , gamma{ gamma }
        , estimate{ initialEstimate }
        , dt{ dt }
    {}

    template<typename T, std::size_t Dof, std::size_t NumParams>
    OPTIMIZE_FOR_SPEED auto SlotineLiAdaptiveControl<T, Dof, NumParams>::ComputeTorque(const StateVector& q, const StateVector& qDot, const StateVector& qd, const StateVector& qdDot,
        const StateVector& qdDdot) -> StateVector
    {
        const StateVector positionError{ q - qd };
        const StateVector velocityError{ qDot - qdDot };
        const StateVector sliding{ velocityError + lambda * positionError };
        const StateVector referenceVelocity{ qdDot - lambda * positionError };
        const StateVector referenceAcceleration{ qdDdot - lambda * velocityError };

        const auto y{ regressor.Compute(q, qDot, referenceVelocity, referenceAcceleration) };
        const StateVector torque{ y * estimate - kd * sliding };
        estimate = estimate - ParameterVector{ gamma * ParameterVector{ y.Transpose() * sliding } } * dt;

        return torque;
    }

    template<typename T, std::size_t Dof, std::size_t NumParams>
    auto SlotineLiAdaptiveControl<T, Dof, NumParams>::ParameterEstimate() const -> const ParameterVector&
    {
        return estimate;
    }

    template<typename T, std::size_t Dof, std::size_t NumParams>
    void SlotineLiAdaptiveControl<T, Dof, NumParams>::Reset(const ParameterVector& initialEstimate)
    {
        estimate = initialEstimate;
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class SlotineLiAdaptiveControl<float, 2, 3>;
    extern template class SlotineLiAdaptiveControl<float, 2, 20>;
#endif
}
