#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/SE3Transform.hpp"
#include "robotics/planning/ConvexCollision.hpp"
#include "robotics/planning/PrimitiveDistance.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <gtest/gtest.h>
#include <limits>
#include <numbers>

namespace
{
    using Vector3 = math::Vector3<float>;
    using Matrix3 = math::Matrix3<float>;
    using Transform = math::SE3Transform<float>;
    using Collision = planning::ConvexCollision<float, 16, 64>;
    using Shape = planning::ConvexShape<float, 16>;
    using Distance = planning::PrimitiveDistance<float>;

    constexpr float pi{ std::numbers::pi_v<float> };

    Matrix3 Rotation(const Vector3& axis, float angle)
    {
        const float norm{ std::sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]) };
        return math::RotationAboutAxis(Vector3{ axis * (1.0f / norm) }, angle);
    }

    float Dot(const Vector3& a, const Vector3& b)
    {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    Vector3 Column(const Matrix3& m, std::size_t c)
    {
        return Vector3{ m.at(0, c), m.at(1, c), m.at(2, c) };
    }

    std::array<Vector3, 8> Corners(const Shape& box)
    {
        std::array<Vector3, 8> corners{};
        for (std::size_t i = 0; i < 8; ++i)
        {
            const Vector3 local{ (i & 1u) != 0u ? box.halfExtents[0] : -box.halfExtents[0], (i & 2u) != 0u ? box.halfExtents[1] : -box.halfExtents[1],
                (i & 4u) != 0u ? box.halfExtents[2] : -box.halfExtents[2] };
            corners[i] = Vector3{ box.pose.translation + box.pose.rotation * local };
        }
        return corners;
    }

    float ExactBoxBox(const Shape& a, const Shape& b)
    {
        const planning::Box<float> boxA{ a.pose, a.halfExtents };
        const planning::Box<float> boxB{ b.pose, b.halfExtents };
        const auto cornersA{ Corners(a) };
        const auto cornersB{ Corners(b) };
        float best{ std::numeric_limits<float>::infinity() };
        for (std::size_t i = 0; i < 8; ++i)
        {
            best = std::min(best, Distance::PointBox(cornersA[i], boxB).distance);
            best = std::min(best, Distance::PointBox(cornersB[i], boxA).distance);
        }

        constexpr std::array<std::array<std::size_t, 2>, 12> edges{ { { 0, 1 }, { 2, 3 }, { 4, 5 }, { 6, 7 }, { 0, 2 }, { 1, 3 }, { 4, 6 }, { 5, 7 }, { 0, 4 }, { 1, 5 }, { 2, 6 }, { 3, 7 } } };
        for (const auto& first : edges)
            for (const auto& second : edges)
                best = std::min(best, Distance::SegmentSegment(planning::Segment<float>{ cornersA[first[0]], cornersA[first[1]] }, planning::Segment<float>{ cornersB[second[0]], cornersB[second[1]] }).distance);
        return best;
    }

    float SatDepth(const Shape& a, const Shape& b)
    {
        std::array<Vector3, 15> axes{};
        std::size_t count{ 0 };
        for (std::size_t i = 0; i < 3; ++i)
        {
            axes[count++] = Column(a.pose.rotation, i);
            axes[count++] = Column(b.pose.rotation, i);
        }
        for (std::size_t i = 0; i < 3; ++i)
            for (std::size_t j = 0; j < 3; ++j)
            {
                const Vector3 cross{ math::CrossProduct(Column(a.pose.rotation, i), Column(b.pose.rotation, j)) };
                const float norm{ std::sqrt(Dot(cross, cross)) };
                if (norm > 1e-6f)
                    axes[count++] = Vector3{ cross * (1.0f / norm) };
            }

        const Vector3 offset{ b.pose.translation - a.pose.translation };
        float depth{ std::numeric_limits<float>::infinity() };
        for (std::size_t k = 0; k < count; ++k)
        {
            float radius{ 0.0f };
            for (std::size_t i = 0; i < 3; ++i)
                radius += a.halfExtents[i] * std::abs(Dot(axes[k], Column(a.pose.rotation, i))) + b.halfExtents[i] * std::abs(Dot(axes[k], Column(b.pose.rotation, i)));
            depth = std::min(depth, radius - std::abs(Dot(axes[k], offset)));
        }
        return depth;
    }

    class TestConvexCollision
        : public ::testing::Test
    {
    protected:
        static void ExpectNear(const Vector3& actual, const Vector3& expected, float tolerance = 1e-5f)
        {
            for (std::size_t i = 0; i < 3; ++i)
                EXPECT_NEAR(actual[i], expected[i], tolerance);
        }

        static bool Finite(const planning::ConvexProximity<float>& result)
        {
            return std::isfinite(result.distance) && std::isfinite(result.normal[0]) && std::isfinite(result.normal[1]) && std::isfinite(result.normal[2]);
        }

        float Uniform()
        {
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            return static_cast<float>(static_cast<double>(state) / 4294967296.0);
        }

        float Between(float low, float high)
        {
            return low + (high - low) * Uniform();
        }

        Shape RandomBox()
        {
            const Vector3 axis{ Between(-1.0f, 1.0f), Between(-1.0f, 1.0f), Between(-1.0f, 1.0f) + 1e-3f };
            const Matrix3 rotation{ Rotation(axis, Between(0.0f, pi)) };
            const Vector3 centre{ Between(-0.6f, 0.6f), Between(-0.6f, 0.6f), Between(-0.6f, 0.6f) };
            return Shape::Box(Transform{ rotation, centre }, Vector3{ Between(0.2f, 1.0f), Between(0.2f, 1.0f), Between(0.2f, 1.0f) });
        }

        std::uint32_t state{ 2463534242u };
        Shape boxA{ Shape::Box(Transform{ Matrix3::Identity(), Vector3{} }, Vector3{ 0.5f, 0.4f, 0.3f }) };
        Shape boxB{ Shape::Box(Transform{ Rotation(Vector3{ 1.0f, 1.0f, 0.0f }, 0.7f), Vector3{ 1.4f, 0.6f, 0.5f } }, Vector3{ 0.3f, 0.3f, 0.2f }) };
        Shape cube{ Shape::Box(Transform{ Matrix3::Identity(), Vector3{} }, Vector3{ 1.0f, 1.0f, 1.0f }) };
    };
}

TEST_F(TestConvexCollision, rounded_shapes_match_the_closed_forms)
{
    const auto spheres{ Collision::Query(Shape::Sphere(Vector3{}, 1.0f), Shape::Sphere(Vector3{ 3.0f, 1.0f, 0.0f }, 0.5f)) };
    const auto capsules{ Collision::Query(Shape::Capsule(Vector3{}, Vector3{ 1.0f, 2.0f, 3.0f }, 0.1f), Shape::Capsule(Vector3{ 2.0f, 0.0f, 1.0f }, Vector3{ 0.0f, 1.0f, -1.0f }, 0.2f)) };
    const auto sphereBox{ Collision::Query(Shape::Sphere(Vector3{ 2.0f, 1.0f, 0.0f }, 0.3f),
        Shape::Box(Transform{ Rotation(Vector3{ 0.0f, 0.0f, 1.0f }, pi / 6.0f), Vector3{ 1.0f, 0.0f, 0.0f } }, Vector3{ 0.5f, 0.25f, 0.1f })) };

    EXPECT_NEAR(spheres.distance, 1.662278f, 1e-5f);
    ExpectNear(spheres.onFirst, Vector3{ 0.948683f, 0.316228f, 0.0f });
    EXPECT_NEAR(capsules.distance, 0.648683f, 1e-5f);
    EXPECT_NEAR(sphereBox.distance, 0.573763f, 1e-5f);
    EXPECT_NEAR(Collision::Query(Shape::Capsule(Vector3{ 3.0f, 1.0f, 0.0f }, Vector3{ 3.0f, 1.0f, 0.0f }, 0.5f), Shape::Sphere(Vector3{}, 1.0f)).distance, 1.662278f, 1e-5f);
}

TEST_F(TestConvexCollision, box_pair_distance_matches_the_exact_value)
{
    const auto result{ Collision::Query(boxA, boxB) };

    EXPECT_NEAR(result.distance, 0.511063f, 1e-5f);
    EXPECT_NEAR(result.distance, ExactBoxBox(boxA, boxB), 1e-5f);
    ExpectNear(result.onFirst, Vector3{ 0.5f, 0.391106f, 0.3f });
    ExpectNear(result.onSecond, Vector3{ 1.008894f, 0.391106f, 0.347032f });
}

TEST_F(TestConvexCollision, capsule_box_and_hull_pairs)
{
    const std::array<Vector3, 4> tetrahedron{ Vector3{}, Vector3{ 1.0f, 0.0f, 0.0f }, Vector3{ 0.0f, 1.0f, 0.0f }, Vector3{ 0.0f, 0.0f, 1.0f } };

    const auto capsuleBox{ Collision::Query(Shape::Capsule(Vector3{ 1.0f, -0.5f, 0.8f }, Vector3{ 1.0f, 0.5f, 0.8f }, 0.1f), boxA) };
    const auto hullSphere{ Collision::Query(Shape::Hull(Transform{ Matrix3::Identity(), Vector3{ 2.0f, 0.0f, 0.0f } }, tetrahedron), Shape::Sphere(Vector3{}, 1.0f)) };

    EXPECT_NEAR(capsuleBox.distance, 0.607107f, 1e-5f);
    EXPECT_NEAR(hullSphere.distance, 1.0f, 1e-5f);
}

TEST_F(TestConvexCollision, rounded_overlap_with_separated_cores_is_exact)
{
    const auto result{ Collision::Query(cube, Shape::Sphere(Vector3{ 1.8f, 0.0f, 0.0f }, 1.0f)) };

    EXPECT_NEAR(result.distance, -0.2f, 1e-5f);
    ExpectNear(result.normal, Vector3{ 1.0f, 0.0f, 0.0f });
    EXPECT_TRUE(result.converged);

    const auto coresIntersect{ Collision::Query(cube, Shape::Sphere(Vector3{ 0.5f, 0.0f, 0.0f }, 0.25f)) };
    EXPECT_NEAR(coresIntersect.distance, -0.75f, 1e-3f);
    ExpectNear(coresIntersect.normal, Vector3{ 1.0f, 0.0f, 0.0f }, 1e-2f);
}

TEST_F(TestConvexCollision, epa_resolves_axis_aligned_boxes)
{
    const auto result{ Collision::Query(cube, Shape::Box(Transform{ Matrix3::Identity(), Vector3{ 1.5f, 0.0f, 0.0f } }, Vector3{ 1.0f, 1.0f, 1.0f })) };

    EXPECT_NEAR(result.distance, -0.5f, 1e-5f);
    ExpectNear(result.normal, Vector3{ 1.0f, 0.0f, 0.0f });
}

TEST_F(TestConvexCollision, epa_matches_separating_axes_for_rotated_boxes)
{
    const auto overlapping{ Shape::Box(Transform{ Rotation(Vector3{ 1.0f, 1.0f, 0.0f }, 0.7f), Vector3{ 0.6f, 0.3f, 0.2f } }, Vector3{ 0.3f, 0.3f, 0.2f }) };

    const auto result{ Collision::Query(boxA, overlapping) };

    EXPECT_NEAR(result.distance, -0.291106f, 1e-4f);
    EXPECT_NEAR(result.distance, -SatDepth(boxA, overlapping), 1e-4f);
    ExpectNear(result.normal, Vector3{ 1.0f, 0.0f, 0.0f }, 1e-4f);
}

TEST_F(TestConvexCollision, symmetric_and_tied_configurations_stay_finite)
{
    const auto identical{ Collision::Query(boxA, boxA) };

    EXPECT_NEAR(identical.distance, -0.6f, 1e-5f);
    EXPECT_NEAR(std::abs(identical.normal[2]), 1.0f, 1e-5f);
    EXPECT_TRUE(identical.converged);

    constexpr std::array<float, 3> extents{ 0.25f, 0.5f, 0.75f };
    constexpr std::array<float, 3> offsets{ -0.25f, 0.0f, 0.25f };
    for (std::size_t sample = 0; sample < 50; ++sample)
    {
        const auto pick{ [this](const std::array<float, 3>& values)
            {
                return values[std::min<std::size_t>(static_cast<std::size_t>(Uniform() * 3.0f), 2)];
            } };
        const auto first{ Shape::Box(Transform{ Matrix3::Identity(), Vector3{ pick(offsets), pick(offsets), pick(offsets) } }, Vector3{ pick(extents), pick(extents), pick(extents) }) };
        const auto second{ Shape::Box(Transform{ Matrix3::Identity(), Vector3{ pick(offsets), pick(offsets), pick(offsets) } }, Vector3{ pick(extents), pick(extents), pick(extents) }) };

        const auto result{ Collision::Query(first, second) };

        ASSERT_TRUE(Finite(result)) << "sample " << sample;
        EXPECT_NEAR(-result.distance, SatDepth(first, second), 1e-5f) << "sample " << sample;
    }

    const std::array<Vector3, 3> triangle{ Vector3{}, Vector3{ 1.0f, 0.0f, 0.0f }, Vector3{ 0.0f, 1.0f, 0.0f } };
    const auto flat{ Collision::Query(Shape::Hull(Transform::Identity(), triangle), Shape::Sphere(Vector3{ 0.2f, 0.2f, 0.0f }, 0.0f)) };
    EXPECT_TRUE(Finite(flat));
    EXPECT_FALSE(flat.converged);

    const auto budget{ planning::ConvexCollision<float, 16, 8>::Query(cube, Shape::Sphere(Vector3{ 0.5f, 0.0f, 0.0f }, 0.25f)) };
    EXPECT_TRUE(Finite(budget));
    EXPECT_FALSE(budget.converged);
}

TEST_F(TestConvexCollision, random_box_pairs_agree_with_exact_references)
{
    for (std::size_t sample = 0; sample < 300; ++sample)
    {
        const auto first{ RandomBox() };
        const auto second{ RandomBox() };
        const float depth{ SatDepth(first, second) };

        const auto result{ Collision::Query(first, second) };

        EXPECT_TRUE(result.converged) << "sample " << sample;
        if (depth < 0.0f)
            EXPECT_NEAR(result.distance, ExactBoxBox(first, second), 1e-5f) << "sample " << sample;
        else
            EXPECT_NEAR(-result.distance, depth, 1e-4f) << "sample " << sample;
    }
}
