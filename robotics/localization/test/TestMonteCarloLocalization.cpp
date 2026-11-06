#include "numerical/math/SE2Transform.hpp"
#include "robotics/localization/MonteCarloLocalization.hpp"
#include "robotics/localization/OccupancyGridMap.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    using Transform = math::SE2Transform<float>;
    using Point = math::Vector2<float>;
    using Beam = localization::RangeBeam<float>;
    using Cell = localization::GridCell;
    using Room = localization::OccupancyGridMap<float, 40, 40>;
    using Field = localization::LikelihoodField<float, 40, 40>;
    using Tracker = localization::MonteCarloLocalization<float, 4, 40, 40>;
    using Lattice = localization::MonteCarloLocalization<float, 27, 40, 40>;

    constexpr float pi{ std::numbers::pi_v<float> };
    const Transform truth{ Transform::FromPose(1.0f, 1.5f, 0.2f) };
    const localization::OccupancyGridParameters<float> gridParameters{ 0.1f, Point{ 0.0f, 0.0f }, 0.85f, -0.4f, -2.0f, 3.5f, 4.0f };

    float Zero()
    {
        return 0.0f;
    }

    float One()
    {
        return 1.0f;
    }

    float Half()
    {
        return 0.5f;
    }

    Transform LatticePose(std::size_t i)
    {
        const float dx{ 0.2f * (static_cast<float>(i % 3) - 1.0f) };
        const float dy{ 0.2f * (static_cast<float>((i / 3) % 3) - 1.0f) };
        const float dtheta{ 0.1f * (static_cast<float>(i / 9) - 1.0f) };
        return truth * Transform::FromPose(dx, dy, dtheta);
    }

    class TestMonteCarloLocalization
        : public ::testing::Test
    {
    protected:
        TestMonteCarloLocalization()
        {
            for (int k = 0; k < 40; ++k)
            {
                Occupy(k, 0);
                Occupy(k, 39);
                Occupy(0, k);
                Occupy(39, k);
            }
            for (int y = 0; y <= 14; ++y)
                Occupy(20, y);
            for (int x = 25; x <= 39; ++x)
                Occupy(x, 28);
        }

        void Occupy(int x, int y)
        {
            room.SetLogOdds(Cell{ x, y }, 3.5f);
        }

        std::array<Beam, 12> Scan() const
        {
            std::array<Beam, 12> scan{};
            for (std::size_t k = 0; k < scan.size(); ++k)
            {
                const float bearing{ static_cast<float>(k) * pi / 6.0f };
                const float heading{ truth.Heading() + bearing };
                float range{ 0.0f };
                while (range < 4.0f)
                {
                    const Point end{ truth.translation[0] + range * std::cos(heading), truth.translation[1] + range * std::sin(heading) };
                    const Cell cell{ room.WorldToCell(end) };
                    if (room.Contains(cell) && room.IsOccupied(cell, 0.0f))
                        break;
                    range += 0.005f;
                }
                scan[k] = Beam{ std::min(range, 4.0f), bearing };
            }
            return scan;
        }

        static void ExpectPose(const Transform& actual, float x, float y, float heading, float tolerance)
        {
            const auto pose{ actual.Pose() };
            EXPECT_NEAR(pose[0], x, tolerance);
            EXPECT_NEAR(pose[1], y, tolerance);
            EXPECT_NEAR(pose[2], heading, tolerance);
        }

        Room room{ gridParameters };
        localization::MclParameters<float> parameters{ { 0.05f, 0.01f, 0.05f, 0.01f }, 0.1f, 0.9f, 0.1f, 4.0f, 0.5f, 1 };
    };
}

TEST_F(TestMonteCarloLocalization, likelihood_field_follows_the_chamfer_metric)
{
    Room single{ gridParameters };
    single.SetLogOdds(Cell{ 20, 20 }, 3.5f);
    const Field field{ single, 0.0f, 2.0f };

    EXPECT_NEAR(field.Distance(single.CellCenter(Cell{ 20, 20 })), 0.0f, 1e-6f);
    EXPECT_NEAR(field.Distance(single.CellCenter(Cell{ 25, 20 })), 0.5f, 1e-6f);
    EXPECT_NEAR(field.Distance(single.CellCenter(Cell{ 24, 24 })), 0.565685f, 1e-6f);
    EXPECT_NEAR(field.Distance(single.CellCenter(Cell{ 23, 24 })), 0.524264f, 1e-6f);
    EXPECT_EQ(field.Distance(Point{ -1.0f, 2.0f }), 2.0f);
}

TEST_F(TestMonteCarloLocalization, predict_without_noise_applies_the_increment)
{
    const Field field{ room, 0.0f, 2.0f };
    Tracker tracker{ field, Transform::Identity(), parameters };
    tracker.InitializeAround(Transform::FromPose(1.0f, 2.0f, 0.3f), math::Vector3<float>{ 0.1f, 0.1f, 0.1f }, Zero);

    tracker.Predict(Transform::FromPose(0.5f, 0.0f, 0.1f), Zero);

    for (const auto& particle : tracker.Particles().Particles())
        ExpectPose(Transform::FromPose(particle[0], particle[1], particle[2]), 1.477668f, 2.147760f, 0.4f, 1e-5f);
}

TEST_F(TestMonteCarloLocalization, predict_noise_scales_with_the_motion)
{
    const Field field{ room, 0.0f, 2.0f };
    Tracker tracker{ field, Transform::Identity(), parameters };
    tracker.InitializeAround(Transform::FromPose(1.0f, 2.0f, 0.3f), math::Vector3<float>{ 0.1f, 0.1f, 0.1f }, Zero);

    tracker.Predict(Transform::FromPose(0.5f, 0.0f, 0.1f), One);

    for (const auto& particle : tracker.Particles().Particles())
        ExpectPose(Transform::FromPose(particle[0], particle[1], particle[2]), 1.551732f, 2.288168f, 0.454772f, 1e-5f);
}

TEST_F(TestMonteCarloLocalization, scan_update_favours_the_true_pose)
{
    const Field field{ room, 0.0f, 2.0f };
    parameters.resampleFraction = 0.0f;
    Lattice lattice{ field, Transform::Identity(), parameters };
    lattice.Initialize(LatticePose);
    const auto scan{ Scan() };

    EXPECT_FALSE(lattice.Update(scan, Half));

    const auto& best{ lattice.Particles().MaximumWeight() };
    ExpectPose(Transform::FromPose(best[0], best[1], best[2]), 1.0f, 1.5f, 0.2f, 1e-6f);
    float bestWeight{ 0.0f };
    for (const float weight : lattice.Particles().Weights())
        bestWeight = std::max(bestWeight, weight);
    EXPECT_NEAR(bestWeight, 0.995028f, 1e-4f);
    ExpectPose(lattice.Estimate(), 1.0f, 1.5f, 0.200381f, 1e-4f);
}

TEST_F(TestMonteCarloLocalization, resampling_concentrates_on_the_true_pose)
{
    const Field field{ room, 0.0f, 2.0f };
    Lattice lattice{ field, Transform::Identity(), parameters };
    lattice.Initialize(LatticePose);
    const auto scan{ Scan() };

    EXPECT_TRUE(lattice.Update(scan, Half));

    for (const auto& particle : lattice.Particles().Particles())
        ExpectPose(Transform::FromPose(particle[0], particle[1], particle[2]), 1.0f, 1.5f, 0.2f, 1e-6f);
    ExpectPose(lattice.Estimate(), 1.0f, 1.5f, 0.2f, 1e-5f);
}

TEST_F(TestMonteCarloLocalization, max_range_beams_are_ignored)
{
    const Field field{ room, 0.0f, 2.0f };
    Lattice lattice{ field, Transform::Identity(), parameters };
    lattice.Initialize(LatticePose);
    std::array<Beam, 12> empty{};
    for (std::size_t k = 0; k < empty.size(); ++k)
        empty[k] = Beam{ 4.0f, static_cast<float>(k) * pi / 6.0f };

    lattice.Update(empty, Half);

    for (const float weight : lattice.Particles().Weights())
        EXPECT_NEAR(weight, 1.0f / 27.0f, 1e-6f);
}
