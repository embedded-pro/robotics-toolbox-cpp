#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/Matrix.hpp"
#include "robotics/dynamics/CoriolisMatrix.hpp"
#include "robotics/dynamics/EulerLagrangeDynamics.hpp"
#include "robotics/dynamics/JointLink.hpp"
#include <array>
#include <cstddef>
#include <type_traits>

namespace dynamics
{
    template<typename T, std::size_t Dof>
    class MomentumObserver
    {
        static_assert(std::is_floating_point_v<T>, "MomentumObserver supports floating-point types");

    public:
        using JointVector = math::Vector<T, Dof>;
        using LinkArray = std::array<JointLink<T>, Dof>;

        MomentumObserver(const EulerLagrangeDynamics<T, Dof>& model, const CoriolisMatrix<T, Dof>& coriolis, const LinkArray& links, const JointVector& gain);

        void Reset(const JointVector& q, const JointVector& qDot);
        OPTIMIZE_FOR_SPEED const JointVector& Update(const JointVector& q, const JointVector& qDot, const JointVector& tauCommanded, T dt);
        bool Collision(const JointVector& threshold) const;
        const JointVector& Residual() const;

    private:
        JointVector Momentum(const JointVector& q, const JointVector& qDot) const;

        const EulerLagrangeDynamics<T, Dof>& model;
        const CoriolisMatrix<T, Dof>& coriolis;
        const LinkArray& links;
        JointVector gain;
        JointVector initialMomentum{};
        JointVector integral{};
        JointVector residual{};
    };

    template<typename T, std::size_t Dof>
    MomentumObserver<T, Dof>::MomentumObserver(const EulerLagrangeDynamics<T, Dof>& model, const CoriolisMatrix<T, Dof>& coriolis, const LinkArray& links, const JointVector& gain)
        : model{ model }
        , coriolis{ coriolis }
        , links{ links }
        , gain{ gain }
    {}

    template<typename T, std::size_t Dof>
    void MomentumObserver<T, Dof>::Reset(const JointVector& q, const JointVector& qDot)
    {
        initialMomentum = Momentum(q, qDot);
        integral = JointVector{};
        residual = JointVector{};
    }

    template<typename T, std::size_t Dof>
    OPTIMIZE_FOR_SPEED const typename MomentumObserver<T, Dof>::JointVector& MomentumObserver<T, Dof>::Update(const JointVector& q, const JointVector& qDot, const JointVector& tauCommanded, T dt)
    {
        const JointVector beta{ model.ComputeGravityTerms(q) - coriolis.TransposeTimesVelocity(links, q, qDot) };
        integral = integral + JointVector{ tauCommanded - beta + residual } * dt;

        const JointVector unfiltered{ Momentum(q, qDot) - initialMomentum - integral };
        for (std::size_t i = 0; i < Dof; ++i)
            residual[i] = gain[i] * unfiltered[i];

        return residual;
    }

    template<typename T, std::size_t Dof>
    bool MomentumObserver<T, Dof>::Collision(const JointVector& threshold) const
    {
        for (std::size_t i = 0; i < Dof; ++i)
            if (math::Abs(residual[i]) > threshold[i])
                return true;

        return false;
    }

    template<typename T, std::size_t Dof>
    const typename MomentumObserver<T, Dof>::JointVector& MomentumObserver<T, Dof>::Residual() const
    {
        return residual;
    }

    template<typename T, std::size_t Dof>
    typename MomentumObserver<T, Dof>::JointVector MomentumObserver<T, Dof>::Momentum(const JointVector& q, const JointVector& qDot) const
    {
        return model.ComputeMassMatrix(q) * qDot;
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class MomentumObserver<float, 2>;
    extern template class MomentumObserver<float, 3>;
#endif
}
