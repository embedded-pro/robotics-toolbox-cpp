#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include "robotics/dynamics/CompositeRigidBodyAlgorithm.hpp"
#include "robotics/dynamics/EulerLagrangeDynamics.hpp"
#include "robotics/dynamics/InverseDynamicsModel.hpp"
#include "robotics/dynamics/JointLink.hpp"
#include "robotics/dynamics/RecursiveNewtonEuler.hpp"
#include <array>
#include <cstddef>
#include <type_traits>

namespace dynamics
{
    template<typename T, std::size_t NumLinks>
    class ChainDynamicsModel
        : public EulerLagrangeDynamics<T, NumLinks>
        , public InverseDynamicsModel<T, NumLinks>
    {
        static_assert(std::is_floating_point_v<T>, "ChainDynamicsModel supports floating-point types");

    public:
        using StateVector = math::Vector<T, NumLinks>;
        using MassMatrix = math::SquareMatrix<T, NumLinks>;
        using Vector3 = math::Vector<T, 3>;
        using LinkArray = std::array<JointLink<T>, NumLinks>;

        ChainDynamicsModel(const LinkArray& links, const Vector3& gravity);

        void SetLinks(const LinkArray& newLinks);

        MassMatrix ComputeMassMatrix(const StateVector& q) const override;
        StateVector ComputeCoriolisTerms(const StateVector& q, const StateVector& qDot) const override;
        StateVector ComputeGravityTerms(const StateVector& q) const override;
        OPTIMIZE_FOR_SPEED StateVector ComputeInverseDynamics(const StateVector& q, const StateVector& qDot, const StateVector& qDDot) const override;

    private:
        LinkArray links;
        Vector3 gravity;
        [[no_unique_address]] RecursiveNewtonEuler<T, NumLinks> rnea;
        [[no_unique_address]] CompositeRigidBodyAlgorithm<T, NumLinks> crba;
    };

    template<typename T, std::size_t NumLinks>
    ChainDynamicsModel<T, NumLinks>::ChainDynamicsModel(const LinkArray& links, const Vector3& gravity)
        : links{ links }
        , gravity{ gravity }
    {}

    template<typename T, std::size_t NumLinks>
    void ChainDynamicsModel<T, NumLinks>::SetLinks(const LinkArray& newLinks)
    {
        links = newLinks;
    }

    template<typename T, std::size_t NumLinks>
    typename ChainDynamicsModel<T, NumLinks>::MassMatrix ChainDynamicsModel<T, NumLinks>::ComputeMassMatrix(const StateVector& q) const
    {
        return crba.Compute(links, q);
    }

    template<typename T, std::size_t NumLinks>
    typename ChainDynamicsModel<T, NumLinks>::StateVector ChainDynamicsModel<T, NumLinks>::ComputeCoriolisTerms(const StateVector& q, const StateVector& qDot) const
    {
        return rnea.InverseDynamics(links, q, qDot, StateVector{}, Vector3{});
    }

    template<typename T, std::size_t NumLinks>
    typename ChainDynamicsModel<T, NumLinks>::StateVector ChainDynamicsModel<T, NumLinks>::ComputeGravityTerms(const StateVector& q) const
    {
        return rnea.InverseDynamics(links, q, StateVector{}, StateVector{}, gravity);
    }

    template<typename T, std::size_t NumLinks>
    OPTIMIZE_FOR_SPEED typename ChainDynamicsModel<T, NumLinks>::StateVector ChainDynamicsModel<T, NumLinks>::ComputeInverseDynamics(const StateVector& q, const StateVector& qDot, const StateVector& qDDot) const
    {
        return rnea.InverseDynamics(links, q, qDot, qDDot, gravity);
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class ChainDynamicsModel<float, 2>;
    extern template class ChainDynamicsModel<float, 3>;
#endif
}
