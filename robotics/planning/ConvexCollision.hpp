#pragma once

#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/CholeskyDecomposition.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/SE3Transform.hpp"
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <span>
#include <type_traits>
#include <utility>

namespace planning
{
    enum class ShapeKind : std::uint8_t
    {
        Sphere,
        Capsule,
        Box,
        Hull
    };

    template<typename T, std::size_t MaxHullVertices = 16>
    struct ConvexShape
    {
        static_assert(std::is_floating_point_v<T>, "ConvexShape supports floating-point types");
        static_assert(MaxHullVertices > 0, "Hulls need at least one vertex slot");

        using Vector3 = math::Vector3<T>;

        ShapeKind kind{ ShapeKind::Sphere };
        math::SE3Transform<T> pose{};
        T radius{};
        Vector3 halfExtents{};
        T halfLength{};
        std::array<Vector3, MaxHullVertices> vertices{};
        std::size_t vertexCount{ 0 };

        static ConvexShape Sphere(const Vector3& centre, T radius);
        static ConvexShape Capsule(const Vector3& start, const Vector3& end, T radius);
        static ConvexShape Box(const math::SE3Transform<T>& pose, const Vector3& halfExtents);
        static ConvexShape Hull(const math::SE3Transform<T>& pose, std::span<const Vector3> points);

        Vector3 CoreSupport(const Vector3& direction) const;
        Vector3 Support(const Vector3& direction) const;
    };

    template<typename T>
    struct ConvexProximity
    {
        static_assert(std::is_floating_point_v<T>, "ConvexProximity supports floating-point types");

        T distance{};
        math::Vector3<T> onFirst{};
        math::Vector3<T> onSecond{};
        math::Vector3<T> normal{};
        bool converged{ false };
        std::size_t iterations{ 0 };
    };

    template<typename T, std::size_t MaxHullVertices = 16, std::size_t MaxFaces = 64>
    class ConvexCollision
    {
        static_assert(std::is_floating_point_v<T>, "ConvexCollision supports floating-point types");
        static_assert(MaxFaces >= 4 && MaxFaces / 2 + 4 <= 255, "Face budget must fit 8-bit vertex indices");

    public:
        using Shape = ConvexShape<T, MaxHullVertices>;

        OPTIMIZE_FOR_SPEED static ConvexProximity<T> Query(const Shape& a, const Shape& b);

    private:
        using Vector3 = math::Vector3<T>;

        static constexpr std::size_t maxGjkIterations{ 32 };
        static constexpr std::size_t maxVertices{ MaxFaces / 2 + 4 };
        static constexpr std::size_t maxEdges{ 3 * MaxFaces };
        static constexpr T relativeTolerance{ T{ 1e-6f } };

        enum class SupportKind : std::uint8_t
        {
            Core,
            Full
        };

        struct Vertex
        {
            Vector3 w{};
            Vector3 onA{};
            Vector3 onB{};
        };

        struct Simplex
        {
            std::array<Vertex, 4> vertices{};
            std::array<T, 4> weights{};
            std::size_t size{ 0 };
        };

        struct GjkResult
        {
            Simplex simplex{};
            Vector3 closest{};
            bool intersecting{ false };
            std::size_t iterations{ 0 };
        };

        struct Face
        {
            std::array<std::uint8_t, 3> index{};
            Vector3 normal{};
            T distance{};
        };

        struct Edge
        {
            std::uint8_t from{};
            std::uint8_t to{};
        };

        struct Polytope
        {
            std::array<Vertex, maxVertices> vertices{};
            std::size_t vertexCount{ 0 };
            std::array<Face, MaxFaces> faces{};
            std::size_t faceCount{ 0 };
            Vector3 centre{};
        };

        static T Dot(const Vector3& lhs, const Vector3& rhs);
        static Vertex MinkowskiSupport(const Shape& a, const Shape& b, const Vector3& direction, SupportKind kind);
        static GjkResult Gjk(const Shape& a, const Shape& b, SupportKind kind);
        static bool ClosestOnSimplex(Simplex& simplex, Vector3& closest);
        static std::optional<std::array<T, 4>> AffineWeights(const std::array<Vertex, 4>& points, std::size_t count);
        static ConvexProximity<T> Separated(const GjkResult& core, const Shape& a, const Shape& b);
        static ConvexProximity<T> Epa(const Shape& a, const Shape& b, const Simplex& seed);
        static bool BuildTetrahedron(const Shape& a, const Shape& b, const Simplex& seed, Polytope& polytope);
        static bool RaisesRank(const Polytope& polytope, const Vector3& point);
        static bool AddFace(Polytope& polytope, std::uint8_t i, std::uint8_t j, std::uint8_t k);
        static bool Expand(Polytope& polytope, const Vertex& support, T scale);
        static std::size_t ClosestFace(const Polytope& polytope);
        static ConvexProximity<T> Contact(const Polytope& polytope, std::size_t face, bool converged, std::size_t iterations);
    };

    template<typename T, std::size_t MaxHullVertices>
    auto ConvexShape<T, MaxHullVertices>::Sphere(const Vector3& centre, T radius) -> ConvexShape
    {
        ConvexShape shape{};
        shape.kind = ShapeKind::Sphere;
        shape.pose.translation = centre;
        shape.radius = radius;
        return shape;
    }

    template<typename T, std::size_t MaxHullVertices>
    auto ConvexShape<T, MaxHullVertices>::Capsule(const Vector3& start, const Vector3& end, T radius) -> ConvexShape
    {
        ConvexShape shape{};
        shape.kind = ShapeKind::Capsule;
        shape.radius = radius;
        shape.pose.translation = Vector3{ (start + end) * T{ 0.5f } };

        const Vector3 axis{ end - start };
        const T length{ math::VectorNorm(axis) };
        if (length <= std::numeric_limits<T>::epsilon())
            return shape;

        shape.halfLength = length / T{ 2 };
        const Vector3 z{ axis * (T{ 1 } / length) };
        const Vector3 helper{ math::Abs(z[0]) < T{ 0.9f } ? Vector3{ T{ 1 }, T{}, T{} } : Vector3{ T{}, T{ 1 }, T{} } };
        const Vector3 projected{ helper - z * (helper[0] * z[0] + helper[1] * z[1] + helper[2] * z[2]) };
        const Vector3 x{ projected * (T{ 1 } / math::VectorNorm(projected)) };
        const Vector3 y{ math::CrossProduct(z, x) };
        shape.pose.rotation = math::Matrix3<T>{ { x[0], y[0], z[0] }, { x[1], y[1], z[1] }, { x[2], y[2], z[2] } };
        return shape;
    }

    template<typename T, std::size_t MaxHullVertices>
    auto ConvexShape<T, MaxHullVertices>::Box(const math::SE3Transform<T>& pose, const Vector3& halfExtents) -> ConvexShape
    {
        ConvexShape shape{};
        shape.kind = ShapeKind::Box;
        shape.pose = pose;
        shape.halfExtents = halfExtents;
        return shape;
    }

    template<typename T, std::size_t MaxHullVertices>
    auto ConvexShape<T, MaxHullVertices>::Hull(const math::SE3Transform<T>& pose, std::span<const Vector3> points) -> ConvexShape
    {
        really_assert(!points.empty() && points.size() <= MaxHullVertices);

        ConvexShape shape{};
        shape.kind = ShapeKind::Hull;
        shape.pose = pose;
        shape.vertexCount = points.size();
        for (std::size_t i = 0; i < points.size(); ++i)
            shape.vertices[i] = points[i];
        return shape;
    }

    template<typename T, std::size_t MaxHullVertices>
    auto ConvexShape<T, MaxHullVertices>::CoreSupport(const Vector3& direction) const -> Vector3
    {
        const Vector3 local{ pose.rotation.Transpose() * direction };
        Vector3 offset{};
        if (kind == ShapeKind::Capsule)
            offset = Vector3{ T{}, T{}, local[2] >= T{} ? halfLength : -halfLength };
        else if (kind == ShapeKind::Box)
            for (std::size_t i = 0; i < 3; ++i)
                offset[i] = local[i] >= T{} ? halfExtents[i] : -halfExtents[i];
        else if (kind == ShapeKind::Hull)
        {
            std::size_t best{ 0 };
            T bestProjection{ -std::numeric_limits<T>::infinity() };
            for (std::size_t v = 0; v < vertexCount; ++v)
            {
                const T projection{ vertices[v][0] * local[0] + vertices[v][1] * local[1] + vertices[v][2] * local[2] };
                if (projection > bestProjection)
                {
                    best = v;
                    bestProjection = projection;
                }
            }
            offset = vertices[best];
        }

        return Vector3{ pose.translation + pose.rotation * offset };
    }

    template<typename T, std::size_t MaxHullVertices>
    auto ConvexShape<T, MaxHullVertices>::Support(const Vector3& direction) const -> Vector3
    {
        const T norm{ math::VectorNorm(direction) };
        if (norm <= T{} || radius == T{})
            return CoreSupport(direction);

        return Vector3{ CoreSupport(direction) + direction * (radius / norm) };
    }

    template<typename T, std::size_t MaxHullVertices, std::size_t MaxFaces>
    OPTIMIZE_FOR_SPEED ConvexProximity<T> ConvexCollision<T, MaxHullVertices, MaxFaces>::Query(const Shape& a, const Shape& b)
    {
        const auto core{ Gjk(a, b, SupportKind::Core) };
        if (!core.intersecting)
            return Separated(core, a, b);

        return Epa(a, b, Gjk(a, b, SupportKind::Full).simplex);
    }

    template<typename T, std::size_t MaxHullVertices, std::size_t MaxFaces>
    T ConvexCollision<T, MaxHullVertices, MaxFaces>::Dot(const Vector3& lhs, const Vector3& rhs)
    {
        return lhs[0] * rhs[0] + lhs[1] * rhs[1] + lhs[2] * rhs[2];
    }

    template<typename T, std::size_t MaxHullVertices, std::size_t MaxFaces>
    auto ConvexCollision<T, MaxHullVertices, MaxFaces>::MinkowskiSupport(const Shape& a, const Shape& b, const Vector3& direction, SupportKind kind) -> Vertex
    {
        const Vector3 opposite{ direction * T{ -1 } };
        const Vector3 onA{ kind == SupportKind::Core ? a.CoreSupport(direction) : a.Support(direction) };
        const Vector3 onB{ kind == SupportKind::Core ? b.CoreSupport(opposite) : b.Support(opposite) };
        return Vertex{ Vector3{ onA - onB }, onA, onB };
    }

    template<typename T, std::size_t MaxHullVertices, std::size_t MaxFaces>
    auto ConvexCollision<T, MaxHullVertices, MaxFaces>::Gjk(const Shape& a, const Shape& b, SupportKind kind) -> GjkResult
    {
        GjkResult result{};
        result.simplex.vertices[0] = MinkowskiSupport(a, b, Vector3{ T{ 1 }, T{}, T{} }, kind);
        result.simplex.weights[0] = T{ 1 };
        result.simplex.size = 1;
        result.closest = result.simplex.vertices[0].w;
        result.intersecting = Dot(result.closest, result.closest) <= T{ 1e-12f };

        while (!result.intersecting && result.iterations < maxGjkIterations)
        {
            const T squared{ Dot(result.closest, result.closest) };
            ++result.iterations;
            const auto support{ MinkowskiSupport(a, b, Vector3{ result.closest * T{ -1 } }, kind) };
            bool duplicate{ false };
            for (std::size_t i = 0; i < result.simplex.size; ++i)
                duplicate = duplicate || (support.w[0] == result.simplex.vertices[i].w[0] && support.w[1] == result.simplex.vertices[i].w[1] && support.w[2] == result.simplex.vertices[i].w[2]);
            if (duplicate || squared - Dot(result.closest, support.w) <= relativeTolerance * squared)
                return result;

            result.simplex.vertices[result.simplex.size++] = support;
            result.intersecting = !ClosestOnSimplex(result.simplex, result.closest) || result.simplex.size == 4;
        }

        return result;
    }

    template<typename T, std::size_t MaxHullVertices, std::size_t MaxFaces>
    bool ConvexCollision<T, MaxHullVertices, MaxFaces>::ClosestOnSimplex(Simplex& simplex, Vector3& closest)
    {
        T bestSquared{ std::numeric_limits<T>::infinity() };
        Simplex best{};

        for (std::size_t mask = 1; mask < (std::size_t{ 1 } << simplex.size); ++mask)
        {
            std::array<Vertex, 4> subset{};
            std::size_t count{ 0 };
            for (std::size_t i = 0; i < simplex.size; ++i)
                if ((mask >> i) & 1u)
                    subset[count++] = simplex.vertices[i];

            const auto weights{ AffineWeights(subset, count) };
            if (!weights)
                continue;

            Vector3 point{};
            for (std::size_t i = 0; i < count; ++i)
                point = point + subset[i].w * (*weights)[i];

            const T squared{ Dot(point, point) };
            if (squared < bestSquared)
            {
                bestSquared = squared;
                best = Simplex{ subset, *weights, count };
                closest = point;
            }
        }

        simplex = best;
        return bestSquared > T{ 1e-12f };
    }

    template<typename T, std::size_t MaxHullVertices, std::size_t MaxFaces>
    auto ConvexCollision<T, MaxHullVertices, MaxFaces>::AffineWeights(const std::array<Vertex, 4>& points, std::size_t count) -> std::optional<std::array<T, 4>>
    {
        std::array<T, 4> weights{ T{ 1 } };
        if (count == 1)
            return weights;

        math::SquareMatrix<T, 3> gram{};
        math::Vector<T, 3> rhs{};
        for (std::size_t r = 1; r < count; ++r)
        {
            const Vector3 dr{ points[r].w - points[0].w };
            rhs[r - 1] = -Dot(dr, points[0].w);
            for (std::size_t c = 1; c < count; ++c)
                gram.at(r - 1, c - 1) = Dot(dr, Vector3{ points[c].w - points[0].w });
        }
        for (std::size_t d = count - 1; d < 3; ++d)
            gram.at(d, d) = T{ 1 };

        const auto mu{ math::CholeskyDecomposition<T, 3>::Solve(gram, rhs).value_or(math::Vector<T, 3>{ T{ -1 }, T{ -1 }, T{ -1 } }) };

        T sum{};
        for (std::size_t i = 1; i < count; ++i)
        {
            weights[i] = mu[i - 1];
            sum += weights[i];
        }
        weights[0] = T{ 1 } - sum;

        for (std::size_t i = 0; i < count; ++i)
            if (weights[i] < T{})
                return std::nullopt;
        return weights;
    }

    template<typename T, std::size_t MaxHullVertices, std::size_t MaxFaces>
    ConvexProximity<T> ConvexCollision<T, MaxHullVertices, MaxFaces>::Separated(const GjkResult& core, const Shape& a, const Shape& b)
    {
        Vector3 onA{};
        Vector3 onB{};
        for (std::size_t i = 0; i < core.simplex.size; ++i)
        {
            onA = onA + core.simplex.vertices[i].onA * core.simplex.weights[i];
            onB = onB + core.simplex.vertices[i].onB * core.simplex.weights[i];
        }

        const Vector3 gap{ onB - onA };
        const T coreDistance{ math::VectorNorm(gap) };
        const Vector3 normal{ gap * (T{ 1 } / coreDistance) };
        return ConvexProximity<T>{ coreDistance - a.radius - b.radius, Vector3{ onA + normal * a.radius }, Vector3{ onB - normal * b.radius }, normal, true, core.iterations };
    }

    template<typename T, std::size_t MaxHullVertices, std::size_t MaxFaces>
    ConvexProximity<T> ConvexCollision<T, MaxHullVertices, MaxFaces>::Epa(const Shape& a, const Shape& b, const Simplex& seed)
    {
        Polytope polytope{};
        if (!BuildTetrahedron(a, b, seed, polytope))
            return ConvexProximity<T>{};

        T scale{ T{ 1 } };
        for (std::size_t i = 0; i < polytope.vertexCount; ++i)
            scale = std::max(scale, math::VectorNorm(polytope.vertices[i].w));

        std::size_t iteration{ 0 };
        bool converged{ false };
        bool expanded{ true };
        while (!converged && expanded && iteration < MaxFaces)
        {
            ++iteration;
            const auto& closest{ polytope.faces[ClosestFace(polytope)] };
            const auto support{ MinkowskiSupport(a, b, closest.normal, SupportKind::Full) };
            converged = Dot(support.w, closest.normal) - closest.distance < relativeTolerance * scale;
            expanded = converged || Expand(polytope, support, scale);
        }

        return Contact(polytope, ClosestFace(polytope), converged, iteration);
    }

    template<typename T, std::size_t MaxHullVertices, std::size_t MaxFaces>
    bool ConvexCollision<T, MaxHullVertices, MaxFaces>::BuildTetrahedron(const Shape& a, const Shape& b, const Simplex& seed, Polytope& polytope)
    {
        for (std::size_t i = 0; i < seed.size && polytope.vertexCount < 4; ++i)
            if (RaisesRank(polytope, seed.vertices[i].w))
                polytope.vertices[polytope.vertexCount++] = seed.vertices[i];

        constexpr T diagonal{ T{ 1 } / std::numbers::sqrt3_v<T> };
        const std::array<Vector3, 14> directions{ Vector3{ T{ 1 }, T{}, T{} }, Vector3{ T{ -1 }, T{}, T{} }, Vector3{ T{}, T{ 1 }, T{} }, Vector3{ T{}, T{ -1 }, T{} },
            Vector3{ T{}, T{}, T{ 1 } }, Vector3{ T{}, T{}, T{ -1 } }, Vector3{ diagonal, diagonal, diagonal }, Vector3{ diagonal, diagonal, -diagonal }, Vector3{ diagonal, -diagonal, diagonal },
            Vector3{ diagonal, -diagonal, -diagonal }, Vector3{ -diagonal, diagonal, diagonal }, Vector3{ -diagonal, diagonal, -diagonal }, Vector3{ -diagonal, -diagonal, diagonal },
            Vector3{ -diagonal, -diagonal, -diagonal } };
        for (std::size_t i = 0; i < directions.size() && polytope.vertexCount < 4; ++i)
        {
            const auto support{ MinkowskiSupport(a, b, directions[i], SupportKind::Full) };
            if (RaisesRank(polytope, support.w))
                polytope.vertices[polytope.vertexCount++] = support;
        }

        if (polytope.vertexCount < 4)
            return false;

        polytope.centre = Vector3{ (polytope.vertices[0].w + polytope.vertices[1].w + polytope.vertices[2].w + polytope.vertices[3].w) * T{ 0.25f } };
        return AddFace(polytope, 0, 1, 2) && AddFace(polytope, 0, 1, 3) && AddFace(polytope, 0, 2, 3) && AddFace(polytope, 1, 2, 3);
    }

    template<typename T, std::size_t MaxHullVertices, std::size_t MaxFaces>
    bool ConvexCollision<T, MaxHullVertices, MaxFaces>::RaisesRank(const Polytope& polytope, const Vector3& point)
    {
        constexpr T tolerance{ T{ 1e-6f } };
        const auto& p{ polytope.vertices };
        if (polytope.vertexCount == 0)
            return true;

        const Vector3 d1{ point - p[0].w };
        if (polytope.vertexCount == 1)
            return math::VectorNorm(d1) > tolerance;

        const Vector3 e1{ p[1].w - p[0].w };
        const Vector3 normal{ math::CrossProduct(e1, d1) };
        if (polytope.vertexCount == 2)
            return math::VectorNorm(normal) > tolerance * math::VectorNorm(e1);

        const Vector3 plane{ math::CrossProduct(e1, Vector3{ p[2].w - p[0].w }) };
        return math::Abs(Dot(plane, d1)) > tolerance * math::VectorNorm(plane);
    }

    template<typename T, std::size_t MaxHullVertices, std::size_t MaxFaces>
    bool ConvexCollision<T, MaxHullVertices, MaxFaces>::AddFace(Polytope& polytope, std::uint8_t i, std::uint8_t j, std::uint8_t k)
    {
        const Vector3& pi{ polytope.vertices[i].w };
        Vector3 normal{ math::CrossProduct(Vector3{ polytope.vertices[j].w - pi }, Vector3{ polytope.vertices[k].w - pi }) };
        const T norm{ math::VectorNorm(normal) };
        if (norm <= std::numeric_limits<T>::min())
            return false;

        normal = normal * (T{ 1 } / norm);
        if (Dot(normal, Vector3{ pi - polytope.centre }) < T{})
        {
            normal = normal * T{ -1 };
            std::swap(j, k);
        }

        polytope.faces[polytope.faceCount++] = Face{ { i, j, k }, normal, Dot(normal, pi) };
        return true;
    }

    template<typename T, std::size_t MaxHullVertices, std::size_t MaxFaces>
    bool ConvexCollision<T, MaxHullVertices, MaxFaces>::Expand(Polytope& polytope, const Vertex& support, T scale)
    {
        std::array<bool, MaxFaces> visible{};
        std::array<Edge, maxEdges> horizon{};
        std::size_t edgeCount{ 0 };
        std::size_t kept{ 0 };
        for (std::size_t f = 0; f < polytope.faceCount; ++f)
        {
            const auto& face{ polytope.faces[f] };
            visible[f] = Dot(face.normal, Vector3{ support.w - polytope.vertices[face.index[0]].w }) > -relativeTolerance * scale;
            kept += visible[f] ? 0 : 1;
            for (std::size_t e = 0; visible[f] && e < 3; ++e)
            {
                const Edge edge{ face.index[e], face.index[(e + 1) % 3] };
                std::size_t match{ edgeCount };
                for (std::size_t h = 0; h < edgeCount; ++h)
                    if (horizon[h].from == edge.to && horizon[h].to == edge.from)
                        match = h;

                if (match < edgeCount)
                    horizon[match] = horizon[--edgeCount];
                else
                    horizon[edgeCount++] = edge;
            }
        }

        if (polytope.vertexCount == maxVertices || kept + edgeCount > MaxFaces)
            return false;

        std::size_t slot{ 0 };
        for (std::size_t f = 0; f < polytope.faceCount; ++f)
            if (!visible[f])
                polytope.faces[slot++] = polytope.faces[f];
        polytope.faceCount = slot;

        const auto added{ static_cast<std::uint8_t>(polytope.vertexCount) };
        polytope.vertices[polytope.vertexCount++] = support;
        for (std::size_t h = 0; h < edgeCount; ++h)
            if (!AddFace(polytope, horizon[h].from, horizon[h].to, added))
                return false;

        return true;
    }

    template<typename T, std::size_t MaxHullVertices, std::size_t MaxFaces>
    std::size_t ConvexCollision<T, MaxHullVertices, MaxFaces>::ClosestFace(const Polytope& polytope)
    {
        std::size_t closest{ 0 };
        for (std::size_t f = 1; f < polytope.faceCount; ++f)
            if (polytope.faces[f].distance < polytope.faces[closest].distance)
                closest = f;
        return closest;
    }

    template<typename T, std::size_t MaxHullVertices, std::size_t MaxFaces>
    ConvexProximity<T> ConvexCollision<T, MaxHullVertices, MaxFaces>::Contact(const Polytope& polytope, std::size_t face, bool converged, std::size_t iterations)
    {
        const auto& closest{ polytope.faces[face] };
        const auto& v0{ polytope.vertices[closest.index[0]] };
        const auto& v1{ polytope.vertices[closest.index[1]] };
        const auto& v2{ polytope.vertices[closest.index[2]] };

        const Vector3 point{ closest.normal * closest.distance };
        const Vector3 e1{ v1.w - v0.w };
        const Vector3 e2{ v2.w - v0.w };
        const Vector3 rel{ point - v0.w };
        const T d11{ Dot(e1, e1) };
        const T d12{ Dot(e1, e2) };
        const T d22{ Dot(e2, e2) };
        const T determinant{ d11 * d22 - d12 * d12 };
        const T u{ (d22 * Dot(rel, e1) - d12 * Dot(rel, e2)) / determinant };
        const T v{ (d11 * Dot(rel, e2) - d12 * Dot(rel, e1)) / determinant };
        const T w{ T{ 1 } - u - v };

        return ConvexProximity<T>{ -closest.distance, Vector3{ v0.onA * w + v1.onA * u + v2.onA * v }, Vector3{ v0.onB * w + v1.onB * u + v2.onB * v }, closest.normal, converged, iterations };
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template struct ConvexShape<float, 16>;
    extern template class ConvexCollision<float, 16, 64>;
    extern template class ConvexCollision<float, 16, 8>;
#endif
}
