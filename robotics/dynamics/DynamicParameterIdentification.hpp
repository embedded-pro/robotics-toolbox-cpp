#pragma once

#include "numerical/estimators/offline/IncrementalQrLeastSquares.hpp"
#include "numerical/math/Matrix.hpp"
#include "robotics/dynamics/InertialRegressor.hpp"
#include <cstddef>
#include <optional>
#include <type_traits>

namespace dynamics
{
    template<typename T, std::size_t Parameters>
    struct IdentificationResult
    {
        static_assert(std::is_floating_point_v<T>, "IdentificationResult supports floating-point types");

        math::Vector<T, Parameters> parameters{};
        std::size_t rank{ 0 };
        math::SquareMatrix<T, Parameters> baseDirections{};
        T residualRms{};
    };

    template<typename T, std::size_t Dof>
    class DynamicParameterIdentification
    {
        static_assert(std::is_floating_point_v<T>, "DynamicParameterIdentification supports floating-point types");

    public:
        static constexpr std::size_t ParameterCount{ 10 * Dof };

        using JointVector = math::Vector<T, Dof>;
        using Regressor = InertialRegressor<T, Dof, ParameterCount>;
        using Result = IdentificationResult<T, ParameterCount>;

        explicit DynamicParameterIdentification(const Regressor& regressor);

        void AddSample(const JointVector& q, const JointVector& qDot, const JointVector& qDDot, const JointVector& tauMeasured);
        std::optional<Result> Solve(T relativeTolerance) const;
        JointVector Predict(const Result& result, const JointVector& q, const JointVector& qDot, const JointVector& qDDot) const;
        void Reset();

    private:
        const Regressor& regressor;
        estimators::IncrementalQrLeastSquares<T, ParameterCount> leastSquares;
    };

    template<typename T, std::size_t Dof>
    DynamicParameterIdentification<T, Dof>::DynamicParameterIdentification(const Regressor& regressor)
        : regressor{ regressor }
    {}

    template<typename T, std::size_t Dof>
    void DynamicParameterIdentification<T, Dof>::AddSample(const JointVector& q, const JointVector& qDot, const JointVector& qDDot, const JointVector& tauMeasured)
    {
        leastSquares.AddRows(regressor.Compute(q, qDot, qDot, qDDot), tauMeasured);
    }

    template<typename T, std::size_t Dof>
    auto DynamicParameterIdentification<T, Dof>::Solve(T relativeTolerance) const -> std::optional<Result>
    {
        const auto solution{ leastSquares.Solve(relativeTolerance) };
        if (!solution)
            return std::nullopt;

        return Result{ solution->parameters, solution->rank, solution->rowSpace, solution->residualRms };
    }

    template<typename T, std::size_t Dof>
    auto DynamicParameterIdentification<T, Dof>::Predict(const Result& result, const JointVector& q, const JointVector& qDot, const JointVector& qDDot) const -> JointVector
    {
        return regressor.Compute(q, qDot, qDot, qDDot) * result.parameters;
    }

    template<typename T, std::size_t Dof>
    void DynamicParameterIdentification<T, Dof>::Reset()
    {
        leastSquares.Reset();
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class DynamicParameterIdentification<float, 2>;
#endif
}
