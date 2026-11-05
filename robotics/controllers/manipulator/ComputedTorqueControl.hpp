#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include "robotics/dynamics/InverseDynamicsModel.hpp"
#include <cstddef>
#include <type_traits>

namespace controllers
{
    template<typename T, std::size_t Dof>
    class ComputedTorqueControl
    {
        static_assert(std::is_floating_point_v<T>, "ComputedTorqueControl supports floating-point types");
        static_assert(Dof > 0, "Degrees of freedom must be positive");

    public:
        using StateVector = math::Vector<T, Dof>;
        using GainMatrix = math::SquareMatrix<T, Dof>;

        ComputedTorqueControl(const dynamics::InverseDynamicsModel<T, Dof>& model, const GainMatrix& kp, const GainMatrix& kd);

        OPTIMIZE_FOR_SPEED StateVector ComputeTorque(const StateVector& q, const StateVector& qDot,
            const StateVector& qDesired, const StateVector& qDotDesired, const StateVector& qDDotDesired) const;

    private:
        const dynamics::InverseDynamicsModel<T, Dof>& model;
        GainMatrix kp;
        GainMatrix kd;
    };

    template<typename T, std::size_t Dof>
    ComputedTorqueControl<T, Dof>::ComputedTorqueControl(const dynamics::InverseDynamicsModel<T, Dof>& model, const GainMatrix& kp, const GainMatrix& kd)
        : model{ model }
        , kp{ kp }
        , kd{ kd }
    {}

    template<typename T, std::size_t Dof>
    OPTIMIZE_FOR_SPEED typename ComputedTorqueControl<T, Dof>::StateVector ComputedTorqueControl<T, Dof>::ComputeTorque(const StateVector& q, const StateVector& qDot,
        const StateVector& qDesired, const StateVector& qDotDesired, const StateVector& qDDotDesired) const
    {
        const StateVector acceleration{ qDDotDesired + kd * StateVector{ qDotDesired - qDot } + kp * StateVector{ qDesired - q } };
        return model.ComputeInverseDynamics(q, qDot, acceleration);
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class ComputedTorqueControl<float, 2>;
    extern template class ComputedTorqueControl<float, 3>;
#endif
}
