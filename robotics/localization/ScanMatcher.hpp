#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/SE2Transform.hpp"
#include "numerical/optimization/DampingPolicy.hpp"
#include "numerical/optimization/LeastSquaresProblem.hpp"
#include "numerical/optimization/LevenbergMarquardt.hpp"
#include "numerical/solvers/JacobiEigenSolver.hpp"
#include "numerical/solvers/PointSetAlignment.hpp"
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
    enum class IcpMetric : std::uint8_t
    {
        PointToPoint,
        PointToLine
    };

    template<typename T>
    struct ScanMatcherParameters
    {
        static_assert(std::is_floating_point_v<T>, "ScanMatcherParameters supports floating-point types");

        IcpMetric metric{ IcpMetric::PointToLine };
        T maxCorrespondenceDistance{};
        std::size_t maxIterations{};
        T translationTolerance{};
        T rotationTolerance{};
        std::size_t minCorrespondences{ 3 };
        std::size_t refinementIterations{ 3 };
    };

    template<typename T>
    struct ScanMatchResult
    {
        static_assert(std::is_floating_point_v<T>, "ScanMatchResult supports floating-point types");

        math::SE2Transform<T> transform{};
        std::size_t iterations{};
        std::size_t correspondences{};
        T rms{};
        bool converged{ false };
    };

    template<typename T, std::size_t MaxPoints>
    class ScanMatcher
    {
        static_assert(std::is_floating_point_v<T>, "ScanMatcher supports floating-point types");
        static_assert(MaxPoints >= 3, "ScanMatcher needs room for at least three points");

    public:
        using Point = math::Vector2<T>;

        explicit ScanMatcher(const ScanMatcherParameters<T>& parameters);

        OPTIMIZE_FOR_SPEED std::optional<ScanMatchResult<T>> Match(std::span<const Point> reference, std::span<const Point> current, const math::SE2Transform<T>& initialGuess) const;

    private:
        struct Correspondences
        {
            std::array<Point, MaxPoints> moved{};
            std::array<Point, MaxPoints> matched{};
            std::array<Point, MaxPoints> normals{};
            std::array<T, MaxPoints> weights{};
            std::size_t size{ 0 };
            std::size_t accepted{ 0 };
        };

        class PointToLineProblem
            : public optimization::LeastSquaresProblem<T, math::SE2Transform<T>, MaxPoints, 3>
        {
        public:
            explicit PointToLineProblem(const Correspondences& correspondences);

            math::Vector<T, MaxPoints> Residual(const math::SE2Transform<T>& delta) const override;
            math::Matrix<T, MaxPoints, 3> Jacobian(const math::SE2Transform<T>& delta) const override;
            math::SE2Transform<T> Retract(const math::SE2Transform<T>& delta, const math::Vector<T, 3>& step) const override;

        private:
            const Correspondences& correspondences;
        };

        static constexpr T degeneracyRatio{ T{ 1e-6f } };
        static constexpr T dampingSquared{ T{ 1e-9f } };

        void Associate(std::span<const Point> reference, std::span<const Point> current, const math::SE2Transform<T>& transform, Correspondences& correspondences) const;
        std::size_t NearestSegment(std::span<const Point> reference, const Point& point) const;
        static T SegmentDistance(const Point& from, const Point& to, const Point& point);
        static Point SegmentNormal(const Point& from, const Point& to);
        std::optional<math::SE2Transform<T>> Step(const Correspondences& correspondences) const;
        std::optional<math::SE2Transform<T>> PointToPointStep(const Correspondences& correspondences) const;
        std::optional<math::SE2Transform<T>> PointToLineStep(const Correspondences& correspondences) const;
        T Rms(const Correspondences& correspondences) const;
        ScanMatchResult<T> Result(std::span<const Point> reference, std::span<const Point> current, const math::SE2Transform<T>& transform, std::size_t iterations, bool converged) const;

        ScanMatcherParameters<T> parameters;
    };

    template<typename T, std::size_t MaxPoints>
    ScanMatcher<T, MaxPoints>::ScanMatcher(const ScanMatcherParameters<T>& parameters)
        : parameters{ parameters }
    {}

    template<typename T, std::size_t MaxPoints>
    OPTIMIZE_FOR_SPEED std::optional<ScanMatchResult<T>> ScanMatcher<T, MaxPoints>::Match(std::span<const Point> reference, std::span<const Point> current,
        const math::SE2Transform<T>& initialGuess) const
    {
        if (reference.size() > MaxPoints || current.size() > MaxPoints || reference.size() < 2)
            return std::nullopt;

        auto transform{ initialGuess };
        Correspondences correspondences{};
        for (std::size_t iteration = 1; iteration <= parameters.maxIterations; ++iteration)
        {
            Associate(reference, current, transform, correspondences);
            if (correspondences.accepted < parameters.minCorrespondences)
                return std::nullopt;

            const auto delta{ Step(correspondences) };
            if (!delta)
                return std::nullopt;

            transform = (*delta * transform).Normalized();
            const bool small{ math::Hypot(delta->translation[0], delta->translation[1]) < parameters.translationTolerance && math::Abs(delta->Heading()) < parameters.rotationTolerance };
            if (small)
                return Result(reference, current, transform, iteration, true);
        }

        return Result(reference, current, transform, parameters.maxIterations, false);
    }

    template<typename T, std::size_t MaxPoints>
    void ScanMatcher<T, MaxPoints>::Associate(std::span<const Point> reference, std::span<const Point> current, const math::SE2Transform<T>& transform, Correspondences& correspondences) const
    {
        correspondences.size = current.size();
        correspondences.accepted = 0;
        for (std::size_t i = 0; i < current.size(); ++i)
        {
            const auto moved{ transform.Apply(current[i]) };
            std::size_t nearest{ 0 };
            T nearestDistance{ std::numeric_limits<T>::infinity() };
            for (std::size_t j = 0; j < reference.size(); ++j)
            {
                const T distance{ math::Hypot(reference[j][0] - moved[0], reference[j][1] - moved[1]) };
                if (distance < nearestDistance)
                {
                    nearest = j;
                    nearestDistance = distance;
                }
            }

            const bool accepted{ nearestDistance <= parameters.maxCorrespondenceDistance };
            const std::size_t segment{ NearestSegment(reference, moved) };
            correspondences.moved[i] = moved;
            correspondences.matched[i] = parameters.metric == IcpMetric::PointToPoint ? reference[nearest] : reference[segment];
            correspondences.normals[i] = SegmentNormal(reference[segment], reference[segment + 1]);
            correspondences.weights[i] = accepted ? T{ 1 } : T{};
            correspondences.accepted += accepted ? 1 : 0;
        }
    }

    template<typename T, std::size_t MaxPoints>
    std::size_t ScanMatcher<T, MaxPoints>::NearestSegment(std::span<const Point> reference, const Point& point) const
    {
        std::size_t best{ 0 };
        T bestDistance{ std::numeric_limits<T>::infinity() };
        for (std::size_t k = 0; k + 1 < reference.size(); ++k)
        {
            const bool gap{ math::Hypot(reference[k + 1][0] - reference[k][0], reference[k + 1][1] - reference[k][1]) > parameters.maxCorrespondenceDistance };
            const T distance{ gap ? std::numeric_limits<T>::infinity() : SegmentDistance(reference[k], reference[k + 1], point) };
            if (distance < bestDistance)
            {
                best = k;
                bestDistance = distance;
            }
        }
        return best;
    }

    template<typename T, std::size_t MaxPoints>
    T ScanMatcher<T, MaxPoints>::SegmentDistance(const Point& from, const Point& to, const Point& point)
    {
        const T dx{ to[0] - from[0] };
        const T dy{ to[1] - from[1] };
        const T along{ std::clamp(((point[0] - from[0]) * dx + (point[1] - from[1]) * dy) / (dx * dx + dy * dy), T{}, T{ 1 }) };
        return math::Hypot(point[0] - (from[0] + along * dx), point[1] - (from[1] + along * dy));
    }

    template<typename T, std::size_t MaxPoints>
    auto ScanMatcher<T, MaxPoints>::SegmentNormal(const Point& from, const Point& to) -> Point
    {
        const T dx{ to[0] - from[0] };
        const T dy{ to[1] - from[1] };
        const T length{ math::Hypot(dx, dy) };
        return Point{ -dy / length, dx / length };
    }

    template<typename T, std::size_t MaxPoints>
    std::optional<math::SE2Transform<T>> ScanMatcher<T, MaxPoints>::Step(const Correspondences& correspondences) const
    {
        return parameters.metric == IcpMetric::PointToPoint ? PointToPointStep(correspondences) : PointToLineStep(correspondences);
    }

    template<typename T, std::size_t MaxPoints>
    std::optional<math::SE2Transform<T>> ScanMatcher<T, MaxPoints>::PointToPointStep(const Correspondences& correspondences) const
    {
        const std::span<const Point> moved{ correspondences.moved.data(), correspondences.size };
        const std::span<const Point> matched{ correspondences.matched.data(), correspondences.size };
        const std::span<const T> weights{ correspondences.weights.data(), correspondences.size };
        const auto alignment{ solvers::PointSetAlignment<T, 2>::Align(moved, matched, false, weights) };
        return alignment ? std::optional{ math::SE2Transform<T>::FromPose(alignment->translation[0], alignment->translation[1], math::Atan2(alignment->rotation.at(1, 0), alignment->rotation.at(0, 0))) } : std::nullopt;
    }

    template<typename T, std::size_t MaxPoints>
    std::optional<math::SE2Transform<T>> ScanMatcher<T, MaxPoints>::PointToLineStep(const Correspondences& correspondences) const
    {
        const PointToLineProblem problem{ correspondences };
        const auto jacobian{ problem.Jacobian(math::SE2Transform<T>::Identity()) };
        solvers::JacobiEigenSolver<T, 3> eigen{};
        const bool solved{ eigen.Solve(jacobian.Transpose() * jacobian) };
        if (!solved || eigen.Eigenvalues()[0] <= degeneracyRatio * eigen.Eigenvalues()[2])
            return std::nullopt;

        const optimization::ConstantDamping<T, MaxPoints, 3> damping{ dampingSquared };
        const optimization::LevenbergMarquardt<T, math::SE2Transform<T>, MaxPoints, 3> solver{ problem, damping, { T{}, std::numeric_limits<T>::infinity(), parameters.refinementIterations } };
        return solver.Solve(math::SE2Transform<T>::Identity()).x;
    }

    template<typename T, std::size_t MaxPoints>
    T ScanMatcher<T, MaxPoints>::Rms(const Correspondences& correspondences) const
    {
        T sum{};
        for (std::size_t i = 0; i < correspondences.size; ++i)
        {
            const T dx{ correspondences.moved[i][0] - correspondences.matched[i][0] };
            const T dy{ correspondences.moved[i][1] - correspondences.matched[i][1] };
            const T distance{ parameters.metric == IcpMetric::PointToPoint ? math::Hypot(dx, dy) : correspondences.normals[i][0] * dx + correspondences.normals[i][1] * dy };
            sum += correspondences.weights[i] * distance * distance;
        }
        return math::Sqrt(sum / static_cast<T>(correspondences.accepted));
    }

    template<typename T, std::size_t MaxPoints>
    ScanMatchResult<T> ScanMatcher<T, MaxPoints>::Result(std::span<const Point> reference, std::span<const Point> current, const math::SE2Transform<T>& transform, std::size_t iterations,
        bool converged) const
    {
        Correspondences correspondences{};
        Associate(reference, current, transform, correspondences);
        return ScanMatchResult<T>{ transform, iterations, correspondences.accepted, correspondences.accepted > 0 ? Rms(correspondences) : T{}, converged };
    }

    template<typename T, std::size_t MaxPoints>
    ScanMatcher<T, MaxPoints>::PointToLineProblem::PointToLineProblem(const Correspondences& correspondences)
        : correspondences{ correspondences }
    {}

    template<typename T, std::size_t MaxPoints>
    math::Vector<T, MaxPoints> ScanMatcher<T, MaxPoints>::PointToLineProblem::Residual(const math::SE2Transform<T>& delta) const
    {
        math::Vector<T, MaxPoints> residual{};
        for (std::size_t i = 0; i < correspondences.size; ++i)
        {
            const auto moved{ delta.Apply(correspondences.moved[i]) };
            const auto& normal{ correspondences.normals[i] };
            residual[i] = correspondences.weights[i] * (normal[0] * (moved[0] - correspondences.matched[i][0]) + normal[1] * (moved[1] - correspondences.matched[i][1]));
        }
        return residual;
    }

    template<typename T, std::size_t MaxPoints>
    math::Matrix<T, MaxPoints, 3> ScanMatcher<T, MaxPoints>::PointToLineProblem::Jacobian(const math::SE2Transform<T>& delta) const
    {
        math::Matrix<T, MaxPoints, 3> jacobian{};
        for (std::size_t i = 0; i < correspondences.size; ++i)
        {
            const auto moved{ delta.Apply(correspondences.moved[i]) };
            const auto& normal{ correspondences.normals[i] };
            const T weight{ correspondences.weights[i] };
            jacobian.at(i, 0) = weight * normal[0];
            jacobian.at(i, 1) = weight * normal[1];
            jacobian.at(i, 2) = weight * (normal[1] * moved[0] - normal[0] * moved[1]);
        }
        return jacobian;
    }

    template<typename T, std::size_t MaxPoints>
    math::SE2Transform<T> ScanMatcher<T, MaxPoints>::PointToLineProblem::Retract(const math::SE2Transform<T>& delta, const math::Vector<T, 3>& step) const
    {
        return math::SE2Transform<T>::FromPose(step[0], step[1], step[2]) * delta;
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class ScanMatcher<float, 128>;
#endif
}
