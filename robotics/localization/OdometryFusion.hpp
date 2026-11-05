#pragma once

#include "numerical/filters/active/ResidualKalmanUpdate.hpp"
#include "numerical/math/AngleUtilities.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/MatrixOperations.hpp"
#include "numerical/math/SE2Transform.hpp"
#include <cstddef>
#include <limits>
#include <type_traits>

namespace localization
{
    template<typename T>
    struct OdometryFusionNoise
    {
        static_assert(std::is_floating_point_v<T>, "OdometryFusionNoise supports floating-point types");

        T speedNoiseFloor{};
        T speedNoiseGain{};
        T gyroNoise{};
        T biasRandomWalk{};
        T wheelYawRateNoise{};
        T slipGate{};
    };

    template<typename T>
    class OdometryFusion
    {
        static_assert(std::is_floating_point_v<T>, "OdometryFusion supports floating-point types");

    public:
        using StateVector = math::Vector<T, 4>;
        using CovarianceMatrix = math::SquareMatrix<T, 4>;

        OdometryFusion(const math::SE2Transform<T>& pose, const CovarianceMatrix& covariance, const OdometryFusionNoise<T>& noise);

        OPTIMIZE_FOR_SPEED void Predict(T speed, T gyroRate, T dt);
        filters::KalmanUpdateResult<T> UpdateWheelYawRate(T wheelYawRate, T gyroRate);
        filters::KalmanUpdateResult<T> UpdateHeading(T heading, T sigma);
        filters::KalmanUpdateResult<T> UpdatePosition(const math::Vector2<T>& position, const math::SquareMatrix<T, 2>& positionCovariance);

        math::SE2Transform<T> Pose() const;
        T GyroBias() const;
        const StateVector& State() const;
        const CovarianceMatrix& Covariance() const;

    private:
        template<std::size_t MeasurementSize>
        filters::KalmanUpdateResult<T> Correct(const math::Vector<T, MeasurementSize>& residual, const math::Matrix<T, MeasurementSize, 4>& jacobian,
            const math::SquareMatrix<T, MeasurementSize>& measurementNoise, T gate);

        StateVector state;
        CovarianceMatrix covariance;
        OdometryFusionNoise<T> noise;
    };

    template<typename T>
    OdometryFusion<T>::OdometryFusion(const math::SE2Transform<T>& pose, const CovarianceMatrix& covariance, const OdometryFusionNoise<T>& noise)
        : state{ pose.translation[0], pose.translation[1], pose.Heading(), T{} }
        , covariance{ covariance }
        , noise{ noise }
    {}

    template<typename T>
    OPTIMIZE_FOR_SPEED void OdometryFusion<T>::Predict(T speed, T gyroRate, T dt)
    {
        const T rate{ gyroRate - state[3] };
        const T midpoint{ state[2] + rate * dt / T{ 2 } };
        const T cosine{ math::Cos(midpoint) };
        const T sine{ math::Sin(midpoint) };
        const T arc{ speed * dt };
        const T lever{ speed * dt * dt / T{ 2 } };
        const auto pose{ Pose() * math::SE2Transform<T>::Exp(math::Vector3<T>{ speed, T{}, rate }, dt) };

        auto transition{ CovarianceMatrix::Identity() };
        transition.at(0, 2) = -arc * sine;
        transition.at(0, 3) = lever * sine;
        transition.at(1, 2) = arc * cosine;
        transition.at(1, 3) = -lever * cosine;
        transition.at(2, 3) = -dt;

        const math::Matrix<T, 4, 2> inputMap{ { dt * cosine, -lever * sine }, { dt * sine, lever * cosine }, { T{}, dt }, { T{}, T{} } };
        const T speedSigma{ noise.speedNoiseFloor + noise.speedNoiseGain * math::Abs(speed) };
        const math::SquareMatrix<T, 2> inputNoise{ { speedSigma * speedSigma, T{} }, { T{}, noise.gyroNoise * noise.gyroNoise } };

        covariance = math::CongruenceTransform(transition, covariance) + math::CongruenceTransform(inputMap, inputNoise);
        covariance.at(3, 3) += noise.biasRandomWalk * noise.biasRandomWalk * dt;
        state = StateVector{ pose.translation[0], pose.translation[1], pose.Heading(), state[3] };
    }

    template<typename T>
    filters::KalmanUpdateResult<T> OdometryFusion<T>::UpdateWheelYawRate(T wheelYawRate, T gyroRate)
    {
        const T variance{ noise.wheelYawRateNoise * noise.wheelYawRateNoise + noise.gyroNoise * noise.gyroNoise };
        return Correct<1>(math::Vector<T, 1>{ wheelYawRate - (gyroRate - state[3]) }, math::Matrix<T, 1, 4>{ { T{}, T{}, T{}, T{ -1 } } }, math::SquareMatrix<T, 1>{ { variance } }, noise.slipGate);
    }

    template<typename T>
    filters::KalmanUpdateResult<T> OdometryFusion<T>::UpdateHeading(T heading, T sigma)
    {
        return Correct<1>(math::Vector<T, 1>{ math::AngleDifference(heading, state[2]) }, math::Matrix<T, 1, 4>{ { T{}, T{}, T{ 1 }, T{} } }, math::SquareMatrix<T, 1>{ { sigma * sigma } },
            std::numeric_limits<T>::infinity());
    }

    template<typename T>
    filters::KalmanUpdateResult<T> OdometryFusion<T>::UpdatePosition(const math::Vector2<T>& position, const math::SquareMatrix<T, 2>& positionCovariance)
    {
        const math::Matrix<T, 2, 4> jacobian{ { T{ 1 }, T{}, T{}, T{} }, { T{}, T{ 1 }, T{}, T{} } };
        return Correct<2>(math::Vector<T, 2>{ position[0] - state[0], position[1] - state[1] }, jacobian, positionCovariance, std::numeric_limits<T>::infinity());
    }

    template<typename T>
    math::SE2Transform<T> OdometryFusion<T>::Pose() const
    {
        return math::SE2Transform<T>::FromPose(state[0], state[1], state[2]);
    }

    template<typename T>
    T OdometryFusion<T>::GyroBias() const
    {
        return state[3];
    }

    template<typename T>
    auto OdometryFusion<T>::State() const -> const StateVector&
    {
        return state;
    }

    template<typename T>
    auto OdometryFusion<T>::Covariance() const -> const CovarianceMatrix&
    {
        return covariance;
    }

    template<typename T>
    template<std::size_t MeasurementSize>
    filters::KalmanUpdateResult<T> OdometryFusion<T>::Correct(const math::Vector<T, MeasurementSize>& residual, const math::Matrix<T, MeasurementSize, 4>& jacobian,
        const math::SquareMatrix<T, MeasurementSize>& measurementNoise, T gate)
    {
        const auto result{ filters::ResidualKalmanUpdate<T, 4, MeasurementSize>::Apply(state, covariance, residual, jacobian, measurementNoise, gate) };
        state[2] = math::WrapToPi(state[2]);
        return result;
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class OdometryFusion<float>;
#endif
}
