#pragma once

#include "infra/util/ReallyAssert.hpp"
#include "numerical/filters/active/ResidualKalmanUpdate.hpp"
#include "numerical/math/AngleUtilities.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/MatrixOperations.hpp"
#include "numerical/math/SE2Transform.hpp"
#include "robotics/mobile/PlanarOdometry.hpp"
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <type_traits>

namespace localization
{
    template<typename T>
    struct Landmark
    {
        static_assert(std::is_floating_point_v<T>, "Landmark supports floating-point types");

        math::Vector2<T> position{};
        std::uint16_t id{};
    };

    template<typename T>
    struct RangeBearing
    {
        static_assert(std::is_floating_point_v<T>, "RangeBearing supports floating-point types");

        T range{};
        T bearing{};
        std::optional<std::uint16_t> id{};
    };

    template<typename T>
    struct LandmarkLocalizationNoise
    {
        static_assert(std::is_floating_point_v<T>, "LandmarkLocalizationNoise supports floating-point types");

        mobile::OdometryNoise<T> motion{};
        T rangeNoise{};
        T bearingNoise{};
        T gate{};
    };

    template<typename T>
    struct LandmarkUpdate
    {
        static_assert(std::is_floating_point_v<T>, "LandmarkUpdate supports floating-point types");

        filters::KalmanUpdateResult<T> result{};
        std::optional<std::size_t> landmark{};
    };

    template<typename T, std::size_t MaxLandmarks>
    class LandmarkLocalization
    {
        static_assert(std::is_floating_point_v<T>, "LandmarkLocalization supports floating-point types");
        static_assert(MaxLandmarks > 0, "LandmarkLocalization needs room for at least one landmark");

    public:
        using CovarianceMatrix = math::SquareMatrix<T, 3>;

        LandmarkLocalization(const math::SE2Transform<T>& pose, const CovarianceMatrix& covariance, const LandmarkLocalizationNoise<T>& noise);

        bool SetMap(std::span<const Landmark<T>> landmarks);
        OPTIMIZE_FOR_SPEED void Predict(const math::SE2Transform<T>& increment);
        OPTIMIZE_FOR_SPEED LandmarkUpdate<T> Update(const RangeBearing<T>& observation);
        std::optional<T> Nis(const RangeBearing<T>& observation, std::size_t landmark) const;

        math::SE2Transform<T> Pose() const;
        const CovarianceMatrix& Covariance() const;

    private:
        using Correction = filters::ResidualKalmanUpdate<T, 3, 2>;

        struct Linearization
        {
            typename Correction::Residual residual;
            typename Correction::Jacobian jacobian;
        };

        Linearization Linearize(const RangeBearing<T>& observation, const Landmark<T>& landmark) const;
        std::optional<std::size_t> Associate(const RangeBearing<T>& observation) const;
        std::optional<std::size_t> FindById(std::uint16_t id) const;
        std::optional<std::size_t> Nearest(const RangeBearing<T>& observation) const;
        typename Correction::MeasurementNoise MeasurementNoise() const;

        std::array<Landmark<T>, MaxLandmarks> map{};
        std::size_t count{ 0 };
        math::Vector3<T> state;
        CovarianceMatrix covariance;
        LandmarkLocalizationNoise<T> noise;
    };

    template<typename T, std::size_t MaxLandmarks>
    LandmarkLocalization<T, MaxLandmarks>::LandmarkLocalization(const math::SE2Transform<T>& pose, const CovarianceMatrix& covariance, const LandmarkLocalizationNoise<T>& noise)
        : state{ pose.Pose() }
        , covariance{ covariance }
        , noise{ noise }
    {}

    template<typename T, std::size_t MaxLandmarks>
    bool LandmarkLocalization<T, MaxLandmarks>::SetMap(std::span<const Landmark<T>> landmarks)
    {
        if (landmarks.size() > MaxLandmarks)
            return false;

        std::copy(landmarks.begin(), landmarks.end(), map.begin());
        count = landmarks.size();
        return true;
    }

    template<typename T, std::size_t MaxLandmarks>
    OPTIMIZE_FOR_SPEED void LandmarkLocalization<T, MaxLandmarks>::Predict(const math::SE2Transform<T>& increment)
    {
        const auto delta{ increment.Pose() };
        const T cosine{ math::Cos(state[2]) };
        const T sine{ math::Sin(state[2]) };
        const CovarianceMatrix transition{ { T{ 1 }, T{}, -sine * delta[0] - cosine * delta[1] }, { T{}, T{ 1 }, cosine * delta[0] - sine * delta[1] }, { T{}, T{}, T{ 1 } } };
        const CovarianceMatrix incrementMap{ { cosine, -sine, T{} }, { sine, cosine, T{} }, { T{}, T{}, T{ 1 } } };
        const auto deviation{ mobile::IncrementDeviation(increment, noise.motion) };
        const CovarianceMatrix incrementNoise{ { deviation[0] * deviation[0], T{}, T{} }, { T{}, deviation[1] * deviation[1], T{} }, { T{}, T{}, deviation[2] * deviation[2] } };

        covariance = math::CongruenceTransform(transition, covariance) + math::CongruenceTransform(incrementMap, incrementNoise);
        state = (Pose() * increment).Pose();
    }

    template<typename T, std::size_t MaxLandmarks>
    OPTIMIZE_FOR_SPEED LandmarkUpdate<T> LandmarkLocalization<T, MaxLandmarks>::Update(const RangeBearing<T>& observation)
    {
        const auto landmark{ Associate(observation) };
        if (!landmark)
            return LandmarkUpdate<T>{ filters::KalmanUpdateResult<T>{ filters::KalmanUpdateStatus::Singular, T{} }, std::nullopt };

        const auto linearization{ Linearize(observation, map[*landmark]) };
        const auto result{ Correction::Apply(state, covariance, linearization.residual, linearization.jacobian, MeasurementNoise(), noise.gate) };
        state[2] = math::WrapToPi(state[2]);
        return LandmarkUpdate<T>{ result, landmark };
    }

    template<typename T, std::size_t MaxLandmarks>
    std::optional<T> LandmarkLocalization<T, MaxLandmarks>::Nis(const RangeBearing<T>& observation, std::size_t landmark) const
    {
        really_assert(landmark < count);

        const auto linearization{ Linearize(observation, map[landmark]) };
        return Correction::Nis(linearization.residual, linearization.jacobian, covariance, MeasurementNoise());
    }

    template<typename T, std::size_t MaxLandmarks>
    math::SE2Transform<T> LandmarkLocalization<T, MaxLandmarks>::Pose() const
    {
        return math::SE2Transform<T>::FromPose(state[0], state[1], state[2]);
    }

    template<typename T, std::size_t MaxLandmarks>
    auto LandmarkLocalization<T, MaxLandmarks>::Covariance() const -> const CovarianceMatrix&
    {
        return covariance;
    }

    template<typename T, std::size_t MaxLandmarks>
    auto LandmarkLocalization<T, MaxLandmarks>::Linearize(const RangeBearing<T>& observation, const Landmark<T>& landmark) const -> Linearization
    {
        const T dx{ landmark.position[0] - state[0] };
        const T dy{ landmark.position[1] - state[1] };
        const T squared{ dx * dx + dy * dy };
        const T range{ math::Sqrt(squared) };
        const T bearing{ math::AngleDifference(math::Atan2(dy, dx), state[2]) };

        return Linearization{ typename Correction::Residual{ observation.range - range, math::AngleDifference(observation.bearing, bearing) },
            typename Correction::Jacobian{ { -dx / range, -dy / range, T{} }, { dy / squared, -dx / squared, T{ -1 } } } };
    }

    template<typename T, std::size_t MaxLandmarks>
    std::optional<std::size_t> LandmarkLocalization<T, MaxLandmarks>::Associate(const RangeBearing<T>& observation) const
    {
        return observation.id ? FindById(*observation.id) : Nearest(observation);
    }

    template<typename T, std::size_t MaxLandmarks>
    std::optional<std::size_t> LandmarkLocalization<T, MaxLandmarks>::FindById(std::uint16_t id) const
    {
        const auto first{ map.begin() };
        const auto last{ map.begin() + static_cast<std::ptrdiff_t>(count) };
        const auto found{ std::find_if(first, last, [id](const Landmark<T>& landmark)
            {
                return landmark.id == id;
            }) };
        if (found == last)
            return std::nullopt;
        return static_cast<std::size_t>(found - first);
    }

    template<typename T, std::size_t MaxLandmarks>
    std::optional<std::size_t> LandmarkLocalization<T, MaxLandmarks>::Nearest(const RangeBearing<T>& observation) const
    {
        std::optional<std::size_t> best{};
        T bestNis{ std::numeric_limits<T>::infinity() };
        for (std::size_t k = 0; k < count; ++k)
        {
            const T nis{ Nis(observation, k).value_or(std::numeric_limits<T>::infinity()) };
            if (nis < bestNis)
            {
                best = k;
                bestNis = nis;
            }
        }
        return best;
    }

    template<typename T, std::size_t MaxLandmarks>
    auto LandmarkLocalization<T, MaxLandmarks>::MeasurementNoise() const -> typename Correction::MeasurementNoise
    {
        return typename Correction::MeasurementNoise{ { noise.rangeNoise * noise.rangeNoise, T{} }, { T{}, noise.bearingNoise * noise.bearingNoise } };
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class LandmarkLocalization<float, 8>;
    extern template class LandmarkLocalization<float, 16>;
#endif
}
