#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/SE3Transform.hpp"
#include <algorithm>
#include <cstddef>
#include <type_traits>

namespace planning
{
    template<typename T>
    struct Segment
    {
        static_assert(std::is_floating_point_v<T>, "Segment supports floating-point types");

        math::Vector3<T> start{};
        math::Vector3<T> end{};
    };

    template<typename T>
    struct Sphere
    {
        static_assert(std::is_floating_point_v<T>, "Sphere supports floating-point types");

        math::Vector3<T> center{};
        T radius{};
    };

    template<typename T>
    struct Capsule
    {
        static_assert(std::is_floating_point_v<T>, "Capsule supports floating-point types");

        Segment<T> axis{};
        T radius{};
    };

    template<typename T>
    struct Box
    {
        static_assert(std::is_floating_point_v<T>, "Box supports floating-point types");

        math::SE3Transform<T> pose{};
        math::Vector3<T> halfExtents{};
    };

    template<typename T>
    struct ClosestPoints
    {
        static_assert(std::is_floating_point_v<T>, "ClosestPoints supports floating-point types");

        math::Vector3<T> onFirst{};
        math::Vector3<T> onSecond{};
        T distance{};
    };

    template<typename T>
    struct SegmentPair
    {
        static_assert(std::is_floating_point_v<T>, "SegmentPair supports floating-point types");

        math::Vector3<T> onFirst{};
        math::Vector3<T> onSecond{};
        T s{};
        T t{};
        T distance{};
    };

    template<typename T>
    class PrimitiveDistance
    {
        static_assert(std::is_floating_point_v<T>, "PrimitiveDistance supports floating-point types");

    public:
        using Vector3 = math::Vector3<T>;

        static ClosestPoints<T> PointSegment(const Vector3& point, const Segment<T>& segment);
        OPTIMIZE_FOR_SPEED static SegmentPair<T> SegmentSegment(const Segment<T>& first, const Segment<T>& second);
        static ClosestPoints<T> PointBox(const Vector3& point, const Box<T>& box);
        static ClosestPoints<T> SphereSphere(const Sphere<T>& first, const Sphere<T>& second);
        static ClosestPoints<T> SphereCapsule(const Sphere<T>& sphere, const Capsule<T>& capsule);
        OPTIMIZE_FOR_SPEED static ClosestPoints<T> CapsuleCapsule(const Capsule<T>& first, const Capsule<T>& second);
        static ClosestPoints<T> SphereBox(const Sphere<T>& sphere, const Box<T>& box);

    private:
        struct Parameters
        {
            T s;
            T t;
        };

        struct BoxProjection
        {
            Vector3 surface;
            Vector3 outwardNormal;
            T distance;
            bool inside;
        };

        static constexpr T epsilon{ static_cast<T>(1e-12) };

        static T Clamp01(T value);
        static Parameters SegmentParameters(const Vector3& d1, const Vector3& d2, const Vector3& r);
        static Parameters NonDegenerateParameters(T a, T b, T c, T e, T f);
        static BoxProjection ProjectOnBox(const Vector3& point, const Box<T>& box);
        static BoxProjection ProjectInsideBox(const Vector3& local, const Box<T>& box);
        static ClosestPoints<T> Rounded(const Vector3& firstCore, const Vector3& secondCore, T coreDistance, T firstRadius, T secondRadius);
    };

    template<typename T>
    ClosestPoints<T> PrimitiveDistance<T>::PointSegment(const Vector3& point, const Segment<T>& segment)
    {
        const Vector3 direction{ segment.end - segment.start };
        const T lengthSquared{ math::DotProduct(direction, direction) };
        const T t{ lengthSquared <= epsilon ? T{} : Clamp01(math::DotProduct(point - segment.start, direction) / lengthSquared) };
        const Vector3 closest{ segment.start + direction * t };

        return { point, closest, math::VectorNorm(Vector3{ point - closest }) };
    }

    template<typename T>
    OPTIMIZE_FOR_SPEED SegmentPair<T> PrimitiveDistance<T>::SegmentSegment(const Segment<T>& first, const Segment<T>& second)
    {
        const Vector3 d1{ first.end - first.start };
        const Vector3 d2{ second.end - second.start };
        const auto [s, t]{ SegmentParameters(d1, d2, first.start - second.start) };
        const Vector3 onFirst{ first.start + d1 * s };
        const Vector3 onSecond{ second.start + d2 * t };

        return { onFirst, onSecond, s, t, math::VectorNorm(Vector3{ onSecond - onFirst }) };
    }

    template<typename T>
    ClosestPoints<T> PrimitiveDistance<T>::PointBox(const Vector3& point, const Box<T>& box)
    {
        const auto projection{ ProjectOnBox(point, box) };
        return { point, projection.surface, projection.distance };
    }

    template<typename T>
    ClosestPoints<T> PrimitiveDistance<T>::SphereSphere(const Sphere<T>& first, const Sphere<T>& second)
    {
        return Rounded(first.center, second.center, math::VectorNorm(Vector3{ second.center - first.center }), first.radius, second.radius);
    }

    template<typename T>
    ClosestPoints<T> PrimitiveDistance<T>::SphereCapsule(const Sphere<T>& sphere, const Capsule<T>& capsule)
    {
        const auto core{ PointSegment(sphere.center, capsule.axis) };
        return Rounded(sphere.center, core.onSecond, core.distance, sphere.radius, capsule.radius);
    }

    template<typename T>
    OPTIMIZE_FOR_SPEED ClosestPoints<T> PrimitiveDistance<T>::CapsuleCapsule(const Capsule<T>& first, const Capsule<T>& second)
    {
        const auto core{ SegmentSegment(first.axis, second.axis) };
        return Rounded(core.onFirst, core.onSecond, core.distance, first.radius, second.radius);
    }

    template<typename T>
    ClosestPoints<T> PrimitiveDistance<T>::SphereBox(const Sphere<T>& sphere, const Box<T>& box)
    {
        const auto projection{ ProjectOnBox(sphere.center, box) };
        if (!projection.inside)
            return Rounded(sphere.center, projection.surface, projection.distance, sphere.radius, T{});

        return { sphere.center - projection.outwardNormal * sphere.radius, projection.surface, projection.distance - sphere.radius };
    }

    template<typename T>
    T PrimitiveDistance<T>::Clamp01(T value)
    {
        return std::clamp(value, T{}, T{ 1 });
    }

    template<typename T>
    typename PrimitiveDistance<T>::Parameters PrimitiveDistance<T>::SegmentParameters(const Vector3& d1, const Vector3& d2, const Vector3& r)
    {
        const T a{ math::DotProduct(d1, d1) };
        const T e{ math::DotProduct(d2, d2) };
        const T f{ math::DotProduct(d2, r) };

        if (a <= epsilon && e <= epsilon)
            return { T{}, T{} };
        if (a <= epsilon)
            return { T{}, Clamp01(f / e) };

        const T c{ math::DotProduct(d1, r) };
        if (e <= epsilon)
            return { Clamp01(-c / a), T{} };

        return NonDegenerateParameters(a, math::DotProduct(d1, d2), c, e, f);
    }

    template<typename T>
    typename PrimitiveDistance<T>::Parameters PrimitiveDistance<T>::NonDegenerateParameters(T a, T b, T c, T e, T f)
    {
        const T denominator{ a * e - b * b };
        const T s{ denominator > epsilon ? Clamp01((b * f - c * e) / denominator) : T{} };
        const T t{ (b * s + f) / e };

        if (t < T{})
            return { Clamp01(-c / a), T{} };
        if (t > T{ 1 })
            return { Clamp01((b - c) / a), T{ 1 } };

        return { s, t };
    }

    template<typename T>
    typename PrimitiveDistance<T>::BoxProjection PrimitiveDistance<T>::ProjectOnBox(const Vector3& point, const Box<T>& box)
    {
        const Vector3 local{ box.pose.rotation.Transpose() * (point - box.pose.translation) };

        bool inside{ true };
        Vector3 clamped{};
        for (std::size_t i = 0; i < 3; ++i)
        {
            inside = inside && math::Abs(local[i]) <= box.halfExtents[i];
            clamped[i] = std::clamp(local[i], -box.halfExtents[i], box.halfExtents[i]);
        }

        if (inside)
            return ProjectInsideBox(local, box);

        return { box.pose.Apply(clamped), Vector3{}, math::VectorNorm(Vector3{ local - clamped }), false };
    }

    template<typename T>
    typename PrimitiveDistance<T>::BoxProjection PrimitiveDistance<T>::ProjectInsideBox(const Vector3& local, const Box<T>& box)
    {
        std::size_t face{ 0 };
        for (std::size_t i = 1; i < 3; ++i)
            if (box.halfExtents[i] - math::Abs(local[i]) < box.halfExtents[face] - math::Abs(local[face]))
                face = i;

        const T sign{ local[face] < T{} ? T{ -1 } : T{ 1 } };
        Vector3 surface{ local };
        surface[face] = sign * box.halfExtents[face];
        Vector3 normal{};
        normal[face] = sign;

        return { box.pose.Apply(surface), box.pose.Rotate(normal), math::Abs(local[face]) - box.halfExtents[face], true };
    }

    template<typename T>
    ClosestPoints<T> PrimitiveDistance<T>::Rounded(const Vector3& firstCore, const Vector3& secondCore, T coreDistance, T firstRadius, T secondRadius)
    {
        const Vector3 normal{ coreDistance > math::Sqrt(epsilon) ? Vector3{ (secondCore - firstCore) * (T{ 1 } / coreDistance) } : Vector3{ T{}, T{}, T{ 1 } } };
        return { firstCore + normal * firstRadius, secondCore - normal * secondRadius, coreDistance - firstRadius - secondRadius };
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class PrimitiveDistance<float>;
#endif
}
