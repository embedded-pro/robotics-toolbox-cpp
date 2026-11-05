#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/SE3Transform.hpp"
#include "numerical/optimization/DampingPolicy.hpp"
#include "numerical/optimization/LeastSquaresProblem.hpp"
#include "numerical/optimization/LevenbergMarquardt.hpp"
#include <array>
#include <cstddef>
#include <limits>
#include <type_traits>

namespace kinematics
{
    template<typename T>
    struct StewartGoughGeometry
    {
        static_assert(std::is_floating_point_v<T>, "StewartGoughGeometry supports floating-point types");

        std::array<math::Vector3<T>, 6> baseAnchors{};
        std::array<math::Vector3<T>, 6> platformAnchors{};
    };

    template<typename T>
    struct StewartForwardConfig
    {
        static_assert(std::is_floating_point_v<T>, "StewartForwardConfig supports floating-point types");

        T tolerance{};
        T damping{};
        std::size_t maxIterations{};
    };

    template<typename T>
    struct StewartForwardResult
    {
        static_assert(std::is_floating_point_v<T>, "StewartForwardResult supports floating-point types");

        math::SE3Transform<T> pose{};
        std::size_t iterations{};
        bool converged{ false };
    };

    template<typename T>
    class StewartGoughKinematics
    {
        static_assert(std::is_floating_point_v<T>, "StewartGoughKinematics supports floating-point types");

    public:
        using LegLengths = std::array<T, 6>;

        StewartGoughKinematics(const StewartGoughGeometry<T>& geometry, const StewartForwardConfig<T>& config);

        OPTIMIZE_FOR_SPEED LegLengths Inverse(const math::SE3Transform<T>& pose) const;
        math::Matrix6<T> LegJacobian(const math::SE3Transform<T>& pose) const;
        StewartForwardResult<T> Forward(const LegLengths& lengths, const math::SE3Transform<T>& guess) const;

    private:
        StewartGoughGeometry<T> geometry;
        StewartForwardConfig<T> config;
    };

    template<typename T>
    class StewartLegResidual
        : public optimization::LeastSquaresProblem<T, math::SE3Transform<T>, 6, 6>
    {
        static_assert(std::is_floating_point_v<T>, "StewartLegResidual supports floating-point types");

    public:
        using LegLengths = typename StewartGoughKinematics<T>::LegLengths;

        StewartLegResidual(const StewartGoughKinematics<T>& kinematics, const LegLengths& lengths);

        math::Vector6<T> Residual(const math::SE3Transform<T>& pose) const override;
        math::Matrix6<T> Jacobian(const math::SE3Transform<T>& pose) const override;
        math::SE3Transform<T> Retract(const math::SE3Transform<T>& pose, const math::Vector6<T>& delta) const override;

    private:
        const StewartGoughKinematics<T>& kinematics;
        LegLengths lengths;
    };

    template<typename T>
    StewartGoughKinematics<T>::StewartGoughKinematics(const StewartGoughGeometry<T>& geometry, const StewartForwardConfig<T>& config)
        : geometry{ geometry }
        , config{ config }
    {}

    template<typename T>
    OPTIMIZE_FOR_SPEED auto StewartGoughKinematics<T>::Inverse(const math::SE3Transform<T>& pose) const -> LegLengths
    {
        LegLengths lengths{};
        for (std::size_t i = 0; i < 6; ++i)
            lengths[i] = math::VectorNorm(math::Vector3<T>{ pose.rotation * geometry.platformAnchors[i] + pose.translation - geometry.baseAnchors[i] });
        return lengths;
    }

    template<typename T>
    math::Matrix6<T> StewartGoughKinematics<T>::LegJacobian(const math::SE3Transform<T>& pose) const
    {
        math::Matrix6<T> jacobian{};
        for (std::size_t i = 0; i < 6; ++i)
        {
            const math::Vector3<T> arm{ pose.rotation * geometry.platformAnchors[i] };
            const math::Vector3<T> leg{ arm + pose.translation - geometry.baseAnchors[i] };
            const math::Vector3<T> direction{ leg * (T{ 1 } / math::VectorNorm(leg)) };
            const math::Vector3<T> moment{ math::CrossProduct(arm, direction) };
            for (std::size_t c = 0; c < 3; ++c)
            {
                jacobian.at(i, c) = direction[c];
                jacobian.at(i, c + 3) = moment[c];
            }
        }
        return jacobian;
    }

    template<typename T>
    StewartForwardResult<T> StewartGoughKinematics<T>::Forward(const LegLengths& lengths, const math::SE3Transform<T>& guess) const
    {
        const StewartLegResidual<T> problem{ *this, lengths };
        const optimization::ConstantDamping<T, 6, 6> damping{ config.damping * config.damping };
        const optimization::LevenbergMarquardt<T, math::SE3Transform<T>, 6, 6> solver{ problem, damping, { config.tolerance, std::numeric_limits<T>::infinity(), config.maxIterations } };

        const auto solution{ solver.Solve(guess) };
        return StewartForwardResult<T>{ solution.x, solution.iterations, solution.converged };
    }

    template<typename T>
    StewartLegResidual<T>::StewartLegResidual(const StewartGoughKinematics<T>& kinematics, const LegLengths& lengths)
        : kinematics{ kinematics }
        , lengths{ lengths }
    {}

    template<typename T>
    math::Vector6<T> StewartLegResidual<T>::Residual(const math::SE3Transform<T>& pose) const
    {
        const auto current{ kinematics.Inverse(pose) };
        math::Vector6<T> residual{};
        for (std::size_t i = 0; i < 6; ++i)
            residual[i] = current[i] - lengths[i];
        return residual;
    }

    template<typename T>
    math::Matrix6<T> StewartLegResidual<T>::Jacobian(const math::SE3Transform<T>& pose) const
    {
        return kinematics.LegJacobian(pose);
    }

    template<typename T>
    math::SE3Transform<T> StewartLegResidual<T>::Retract(const math::SE3Transform<T>& pose, const math::Vector6<T>& delta) const
    {
        math::Vector6<T> rotation{};
        for (std::size_t i = 3; i < 6; ++i)
            rotation[i] = delta[i];

        return math::SE3Transform<T>{ math::Matrix3<T>{ math::SE3Transform<T>::Exp(rotation, T{ 1 }).rotation * pose.rotation },
            math::Vector3<T>{ pose.translation + math::Vector3<T>{ delta[0], delta[1], delta[2] } } };
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class StewartGoughKinematics<float>;
    extern template class StewartLegResidual<float>;
#endif
}
