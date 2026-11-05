#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/Tolerance.hpp"
#include "robotics/planning/PrimitiveDistance.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    using Vector3 = math::Vector3<float>;
    using Distance = planning::PrimitiveDistance<float>;
    using Segment = planning::Segment<float>;
    using Sphere = planning::Sphere<float>;
    using Capsule = planning::Capsule<float>;

    class TestPrimitiveDistance
        : public ::testing::Test
    {
    protected:
        void ExpectNear(const Vector3& actual, const Vector3& expected) const
        {
            for (std::size_t i = 0; i < 3; ++i)
                EXPECT_NEAR(actual[i], expected[i], math::Tolerance<float>());
        }

        planning::Box<float> box{ { math::RotationAboutAxis(Vector3{ 0.0f, 0.0f, 1.0f }, std::numbers::pi_v<float> / 6.0f), Vector3{ 1.0f, 0.0f, 0.0f } },
            Vector3{ 0.5f, 0.25f, 0.1f } };
    };
}

TEST_F(TestPrimitiveDistance, point_segment_projects_and_clamps)
{
    const Segment segment{ Vector3{}, Vector3{ 2.0f, 0.0f, 0.0f } };

    const auto interior{ Distance::PointSegment(Vector3{ 1.0f, 1.0f, 0.0f }, segment) };
    const auto clamped{ Distance::PointSegment(Vector3{ -1.0f, 1.0f, 0.0f }, segment) };

    ExpectNear(interior.onSecond, Vector3{ 1.0f, 0.0f, 0.0f });
    EXPECT_NEAR(interior.distance, 1.0f, math::Tolerance<float>());
    ExpectNear(clamped.onSecond, Vector3{});
    EXPECT_NEAR(clamped.distance, std::sqrt(2.0f), math::Tolerance<float>());
}

TEST_F(TestPrimitiveDistance, segment_segment_matches_brute_force)
{
    const auto general{ Distance::SegmentSegment({ Vector3{}, Vector3{ 1.0f, 2.0f, 3.0f } }, { Vector3{ 2.0f, 0.0f, 1.0f }, Vector3{ 0.0f, 1.0f, -1.0f } }) };
    const auto skew{ Distance::SegmentSegment({ Vector3{}, Vector3{ 1.0f, 0.0f, 0.0f } }, { Vector3{ 0.5f, -1.0f, 1.0f }, Vector3{ 0.5f, 1.0f, 1.0f } }) };

    EXPECT_NEAR(general.s, 0.1f, math::Tolerance<float>());
    EXPECT_NEAR(general.t, 0.6f, math::Tolerance<float>());
    ExpectNear(general.onFirst, Vector3{ 0.1f, 0.2f, 0.3f });
    ExpectNear(general.onSecond, Vector3{ 0.8f, 0.6f, -0.2f });
    EXPECT_NEAR(general.distance, 0.948683f, math::Tolerance<float>());
    ExpectNear(skew.onFirst, Vector3{ 0.5f, 0.0f, 0.0f });
    ExpectNear(skew.onSecond, Vector3{ 0.5f, 0.0f, 1.0f });
    EXPECT_NEAR(skew.distance, 1.0f, math::Tolerance<float>());
}

TEST_F(TestPrimitiveDistance, segment_segment_handles_parallel_and_degenerate_inputs)
{
    const auto parallel{ Distance::SegmentSegment({ Vector3{}, Vector3{ 2.0f, 0.0f, 0.0f } }, { Vector3{ 1.0f, 1.0f, 0.0f }, Vector3{ 3.0f, 1.0f, 0.0f } }) };
    const auto endpoints{ Distance::SegmentSegment({ Vector3{}, Vector3{ 1.0f, 0.0f, 0.0f } }, { Vector3{ 2.0f, 1.0f, 0.0f }, Vector3{ 3.0f, 2.0f, 0.0f } }) };
    const auto pointPoint{ Distance::SegmentSegment({ Vector3{}, Vector3{} }, { Vector3{ 1.0f, 1.0f, 1.0f }, Vector3{ 1.0f, 1.0f, 1.0f } }) };
    const auto pointSegment{ Distance::SegmentSegment({ Vector3{ 0.0f, 0.0f, 1.0f }, Vector3{ 0.0f, 0.0f, 1.0f } }, { Vector3{ -1.0f, 0.0f, 0.0f }, Vector3{ 1.0f, 0.0f, 0.0f } }) };
    const auto segmentPoint{ Distance::SegmentSegment({ Vector3{ -1.0f, 0.0f, 0.0f }, Vector3{ 1.0f, 0.0f, 0.0f } }, { Vector3{ 0.0f, 0.0f, 1.0f }, Vector3{ 0.0f, 0.0f, 1.0f } }) };

    EXPECT_NEAR(parallel.distance, 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(endpoints.s, 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(endpoints.t, 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(endpoints.distance, std::sqrt(2.0f), math::Tolerance<float>());
    EXPECT_NEAR(pointPoint.distance, std::sqrt(3.0f), math::Tolerance<float>());
    EXPECT_NEAR(pointSegment.distance, 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(segmentPoint.distance, 1.0f, math::Tolerance<float>());
}

TEST_F(TestPrimitiveDistance, point_box_is_signed)
{
    const auto outside{ Distance::PointBox(Vector3{ 2.0f, 1.0f, 0.0f }, box) };
    const auto inside{ Distance::PointBox(Vector3{ 1.05f, 0.0f, 0.02f }, box) };

    ExpectNear(outside.onSecond, Vector3{ 1.308013f, 0.466506f, 0.0f });
    EXPECT_NEAR(outside.distance, 0.873763f, math::Tolerance<float>());
    ExpectNear(inside.onSecond, Vector3{ 1.05f, 0.0f, 0.1f });
    EXPECT_NEAR(inside.distance, -0.08f, math::Tolerance<float>());
}

TEST_F(TestPrimitiveDistance, rounded_pairs_subtract_the_radii)
{
    const auto spheres{ Distance::SphereSphere({ Vector3{}, 1.0f }, { Vector3{ 3.0f, 1.0f, 0.0f }, 0.5f }) };
    const Capsule first{ { Vector3{}, Vector3{ 1.0f, 0.0f, 0.0f } }, 0.2f };
    const auto apart{ Distance::CapsuleCapsule(first, { { Vector3{ 0.5f, -1.0f, 1.0f }, Vector3{ 0.5f, 1.0f, 1.0f } }, 0.3f }) };
    const auto overlapping{ Distance::CapsuleCapsule(first, { { Vector3{ 0.5f, -1.0f, 0.4f }, Vector3{ 0.5f, 1.0f, 0.4f } }, 0.3f }) };
    const auto sphereCapsule{ Distance::SphereCapsule({ Vector3{ 1.0f, 1.0f, 0.0f }, 0.25f }, { { Vector3{}, Vector3{ 2.0f, 0.0f, 0.0f } }, 0.25f }) };
    const auto sphereBox{ Distance::SphereBox({ Vector3{ 2.0f, 1.0f, 0.0f }, 0.3f }, box) };

    EXPECT_NEAR(spheres.distance, std::sqrt(10.0f) - 1.5f, math::Tolerance<float>());
    ExpectNear(spheres.onFirst, Vector3{ 0.948683f, 0.316228f, 0.0f });
    EXPECT_NEAR(apart.distance, 0.5f, math::Tolerance<float>());
    EXPECT_NEAR(overlapping.distance, -0.1f, math::Tolerance<float>());
    EXPECT_NEAR(sphereCapsule.distance, 0.5f, math::Tolerance<float>());
    EXPECT_NEAR(sphereBox.distance, 0.573763f, math::Tolerance<float>());
}

TEST_F(TestPrimitiveDistance, sphere_inside_a_box_reports_depth)
{
    const auto result{ Distance::SphereBox({ Vector3{ 1.05f, 0.0f, 0.02f }, 0.05f }, box) };

    EXPECT_NEAR(result.distance, -0.13f, math::Tolerance<float>());
}

TEST_F(TestPrimitiveDistance, coincident_cores_still_return_a_normal)
{
    const auto result{ Distance::SphereSphere({ Vector3{ 1.0f, 1.0f, 1.0f }, 0.2f }, { Vector3{ 1.0f, 1.0f, 1.0f }, 0.3f }) };

    EXPECT_NEAR(result.distance, -0.5f, math::Tolerance<float>());
    ExpectNear(result.onFirst, Vector3{ 1.0f, 1.0f, 1.2f });
    ExpectNear(result.onSecond, Vector3{ 1.0f, 1.0f, 0.7f });
}
