#include "robotics/mobile/WheeledInvertedPendulum.hpp"
#include <cstddef>
#include <gtest/gtest.h>

namespace
{
    class TestWheeledInvertedPendulum
        : public ::testing::Test
    {
    protected:
        static mobile::WheeledInvertedPendulumParameters<float> SmallRobot()
        {
            mobile::WheeledInvertedPendulumParameters<float> parameters;
            parameters.bodyMass = 0.8f;
            parameters.wheelsMass = 0.1f;
            parameters.centreOfMassHeight = 0.1f;
            parameters.bodyPitchInertia = 0.004f;
            parameters.wheelInertia = 5e-5f;
            parameters.yawInertia = 0.01f;
            parameters.wheelRadius = 0.034f;
            parameters.trackWidth = 0.145f;
            parameters.gravity = 9.81f;
            return parameters;
        }

        mobile::WheeledInvertedPendulum<float> robot{ SmallRobot() };
    };
}

TEST_F(TestWheeledInvertedPendulum, pitch_plant_matches_the_worked_example)
{
    const auto plant = robot.Pitch();

    EXPECT_NEAR(plant.A.at(0, 1), 1.0f, 1e-6f);
    EXPECT_NEAR(plant.A.at(1, 0), 142.3686f, 1e-2f);
    EXPECT_NEAR(plant.A.at(2, 3), 1.0f, 1e-6f);
    EXPECT_NEAR(plant.A.at(3, 0), -11.54529f, 1e-3f);
    EXPECT_NEAR(plant.B.at(1, 0), -614.0876f, 5e-2f);
    EXPECT_NEAR(plant.B.at(3, 0), 79.61313f, 1e-2f);
}

TEST_F(TestWheeledInvertedPendulum, pitch_plant_has_no_other_terms)
{
    const auto plant = robot.Pitch();

    for (std::size_t row = 0; row < 4; ++row)
        for (std::size_t col = 0; col < 4; ++col)
            if (!(row == 0 && col == 1) && !(row == 1 && col == 0) && !(row == 2 && col == 3) && !(row == 3 && col == 0))
                EXPECT_NEAR(plant.A.at(row, col), 0.0f, 1e-6f);

    EXPECT_NEAR(plant.B.at(0, 0), 0.0f, 1e-6f);
    EXPECT_NEAR(plant.B.at(2, 0), 0.0f, 1e-6f);
}

TEST_F(TestWheeledInvertedPendulum, forward_torque_pitches_the_body_back_and_drives_forward)
{
    const auto plant = robot.Pitch();

    EXPECT_LT(plant.B.at(1, 0), 0.0f);
    EXPECT_GT(plant.B.at(3, 0), 0.0f);
}

TEST_F(TestWheeledInvertedPendulum, unstable_pole_is_an_eigenvalue_of_the_pitch_plant)
{
    const auto plant = robot.Pitch();
    const float pole{ robot.UnstablePole() };
    const float travel{ plant.A.at(3, 0) / (pole * pole) };
    const math::Vector<float, 4> mode{ 1.0f, pole, travel, pole * travel };

    const auto image = plant.A * mode;

    for (std::size_t i = 0; i < 4; ++i)
        EXPECT_NEAR(image[i], pole * mode[i], 1e-3f);
    EXPECT_NEAR(pole, 11.93183f, 1e-3f);
}

TEST_F(TestWheeledInvertedPendulum, fall_time_constant_matches_the_worked_example)
{
    EXPECT_NEAR(robot.FallTimeConstant(), 0.1236548f, 1e-5f);
}

TEST_F(TestWheeledInvertedPendulum, yaw_plant_is_a_double_integrator_scaled_by_the_track)
{
    const auto plant = robot.Yaw();

    EXPECT_NEAR(plant.A.at(0, 0), 0.0f, 1e-6f);
    EXPECT_NEAR(plant.A.at(0, 1), 1.0f, 1e-6f);
    EXPECT_NEAR(plant.A.at(1, 0), 0.0f, 1e-6f);
    EXPECT_NEAR(plant.A.at(1, 1), 0.0f, 1e-6f);
    EXPECT_NEAR(plant.B.at(0, 0), 0.0f, 1e-6f);
    EXPECT_NEAR(plant.B.at(1, 0), 203.9613f, 1e-2f);
}

TEST_F(TestWheeledInvertedPendulum, plants_output_the_full_state)
{
    const auto pitch = robot.Pitch();
    const auto yaw = robot.Yaw();

    for (std::size_t row = 0; row < 4; ++row)
        for (std::size_t col = 0; col < 4; ++col)
            EXPECT_NEAR(pitch.C.at(row, col), row == col ? 1.0f : 0.0f, 1e-6f);
    for (std::size_t row = 0; row < 2; ++row)
        for (std::size_t col = 0; col < 2; ++col)
            EXPECT_NEAR(yaw.C.at(row, col), row == col ? 1.0f : 0.0f, 1e-6f);
}
