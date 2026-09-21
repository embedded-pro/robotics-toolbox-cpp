#include "simulator/dynamics/RobotArm/application/RobotArmForm.hpp"
#include <gmock/gmock.h>
#include <numbers>

namespace
{
    using simulator::dynamics::RobotArmForm;
    namespace field = simulator::dynamics::field;

    class RobotArmFormTest
        : public ::testing::Test
    {
    protected:
        RobotArmForm form;
    };
}

TEST_F(RobotArmFormTest, TheDefaultsMatchThePanelThisReplaced)
{
    const auto config = form.BuildConfiguration();

    EXPECT_EQ(config.dof, 2);
    EXPECT_NEAR(config.damping, 0.05f, 1e-5f);

    ASSERT_EQ(config.linkLengths.size(), 2u);
    EXPECT_NEAR(config.linkLengths[0], 0.5f, 1e-4f);
    EXPECT_NEAR(config.linkLengths[1], 0.4f, 1e-4f);
    EXPECT_NEAR(config.linkMasses[0], 1.0f, 1e-4f);
    EXPECT_NEAR(config.linkMasses[1], 0.8f, 1e-4f);
}

TEST_F(RobotArmFormTest, TheThirdLinkAppearsOnlyForTheSpatialRobot)
{
    EXPECT_FALSE(form.Model().IsGroupVisible(field::thirdLink));
    EXPECT_FALSE(form.Model().IsVisible(field::thirdLength));
    EXPECT_FALSE(form.Model().IsVisible(field::thirdTorque));
    EXPECT_FALSE(form.Model().IsVisible(field::thirdPosition));

    form.Model().SetSelection(field::degreesOfFreedom, 1);

    EXPECT_TRUE(form.Model().IsGroupVisible(field::thirdLink));
    EXPECT_TRUE(form.Model().IsVisible(field::thirdLength));
    EXPECT_TRUE(form.Model().IsVisible(field::thirdTorque));
    EXPECT_TRUE(form.Model().IsVisible(field::thirdPosition));
}

TEST_F(RobotArmFormTest, TheThirdLinkReachesTheConfigurationOnlyWhenSelected)
{
    EXPECT_EQ(form.BuildConfiguration().linkLengths.size(), 2u);

    form.Model().SetSelection(field::degreesOfFreedom, 1);

    const auto config = form.BuildConfiguration();
    ASSERT_EQ(config.linkLengths.size(), 3u);
    EXPECT_EQ(config.dof, 3);
    EXPECT_NEAR(config.linkLengths[2], 0.3f, 1e-4f);
    EXPECT_NEAR(config.linkMasses[2], 0.6f, 1e-4f);
}

TEST_F(RobotArmFormTest, TheDegreeCountComesFromTheOptionDataNotItsPosition)
{
    EXPECT_EQ(form.BuildConfiguration().dof, 2);

    form.Model().SetSelection(field::degreesOfFreedom, 1);

    EXPECT_EQ(form.BuildConfiguration().dof, 3);
}

TEST_F(RobotArmFormTest, TheTorqueSlidersCarryTenthsOfANewtonMetre)
{
    form.Model().SetNumber(field::firstTorque, 125.0);
    form.Model().SetNumber(field::secondTorque, -40.0);

    const auto torques = form.Torques();

    ASSERT_EQ(torques.size(), 2u);
    EXPECT_NEAR(torques[0], 12.5f, 1e-4f);
    EXPECT_NEAR(torques[1], -4.0f, 1e-4f);
}

TEST_F(RobotArmFormTest, ThePositionSlidersAreDegreesAndReachTheSimulatorAsRadians)
{
    form.Model().SetNumber(field::firstPosition, 90.0);
    form.Model().SetNumber(field::secondPosition, -180.0);

    const auto positions = form.InitialPositions();

    ASSERT_EQ(positions.size(), 2u);
    EXPECT_NEAR(positions[0], std::numbers::pi_v<float> / 2.0f, 1e-5f);
    EXPECT_NEAR(positions[1], -std::numbers::pi_v<float>, 1e-5f);
}

TEST_F(RobotArmFormTest, TheTorqueAndPositionVectorsFollowTheSelectedDegreeCount)
{
    EXPECT_EQ(form.Torques().size(), 2u);
    EXPECT_EQ(form.InitialPositions().size(), 2u);

    form.Model().SetSelection(field::degreesOfFreedom, 1);

    EXPECT_EQ(form.Torques().size(), 3u);
    EXPECT_EQ(form.InitialPositions().size(), 3u);
}

TEST_F(RobotArmFormTest, TheDraggedControlsAreSlidersCarryingTheirTickIntervals)
{
    for (const auto id : { field::firstTorque, field::secondTorque, field::thirdTorque })
    {
        EXPECT_EQ(form.Model().Field(id).kind, ui::model::FieldKind::Slider);
        EXPECT_NEAR(form.Model().Field(id).number.tickInterval, 50.0, 1e-9);
    }

    for (const auto id : { field::firstPosition, field::secondPosition, field::thirdPosition })
    {
        EXPECT_EQ(form.Model().Field(id).kind, ui::model::FieldKind::Slider);
        EXPECT_NEAR(form.Model().Field(id).number.tickInterval, 45.0, 1e-9);
    }
}

TEST_F(RobotArmFormTest, TheHiddenThirdLinkKeepsItsValuesAcrossASwitch)
{
    form.Model().SetSelection(field::degreesOfFreedom, 1);
    form.Model().SetNumber(field::thirdLength, 1.25);
    form.Model().SetSelection(field::degreesOfFreedom, 0);
    form.Model().SetSelection(field::degreesOfFreedom, 1);

    EXPECT_NEAR(form.BuildConfiguration().linkLengths[2], 1.25f, 1e-4f);
}
