#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include "robotics/dynamics/EulerLagrangeDynamics.hpp"
#include <cstddef>
#include <type_traits>

namespace controllers
{
    template<typename T, std::size_t Dof>
    class PdGravityCompensation
    {
        static_assert(std::is_floating_point_v<T>, "PdGravityCompensation supports floating-point types");
        static_assert(Dof > 0, "Degrees of freedom must be positive");

    public:
        using StateVector = math::Vector<T, Dof>;
        using GainMatrix = math::SquareMatrix<T, Dof>;

        PdGravityCompensation(const dynamics::EulerLagrangeDynamics<T, Dof>& model, const GainMatrix& kp, const GainMatrix& kd);

        OPTIMIZE_FOR_SPEED StateVector ComputeTorque(const StateVector& q, const StateVector& qDot, const StateVector& qDesired) const;

    private:
        const dynamics::EulerLagrangeDynamics<T, Dof>& model;
        GainMatrix kp;
        GainMatrix kd;
    };

    template<typename T, std::size_t Dof>
    PdGravityCompensation<T, Dof>::PdGravityCompensation(const dynamics::EulerLagrangeDynamics<T, Dof>& model, const GainMatrix& kp, const GainMatrix& kd)
        : model{ model }
        , kp{ kp }
        , kd{ kd }
    {}

    template<typename T, std::size_t Dof>
    OPTIMIZE_FOR_SPEED typename PdGravityCompensation<T, Dof>::StateVector PdGravityCompensation<T, Dof>::ComputeTorque(const StateVector& q, const StateVector& qDot, const StateVector& qDesired) const
    {
        return kp * StateVector{ qDesired - q } - kd * qDot + model.ComputeGravityTerms(q);
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class PdGravityCompensation<float, 2>;
    extern template class PdGravityCompensation<float, 3>;
#endif
}
