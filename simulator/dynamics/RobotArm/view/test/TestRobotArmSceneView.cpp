#include "simulator/dynamics/RobotArm/view/RobotArmSceneView.hpp"
#include "ui/backend/recording/RecordingCanvas.hpp"
#include "ui/theme/Theme.hpp"
#include <gmock/gmock.h>

namespace
{
    using simulator::dynamics::RobotArmState;
    using simulator::dynamics::view::RobotArmSceneView;
    using ui::backend::recording::CommandKind;

    RobotArmState ThreeJointArm(std::size_t trailPoints)
    {
        RobotArmState state;
        state.jointPositions = { { 0.0f, 0.0f, 0.0f }, { 0.4f, 0.0f, 0.2f }, { 0.7f, 0.1f, 0.3f } };
        state.q = { 0.1f, 0.2f };
        state.qDot = { 0.3f, 0.4f };
        state.inverseDynamicsTorques = { 1.5f, -0.5f };
        state.time = 1.25f;

        for (std::size_t i = 0; i < trailPoints; ++i)
        {
            const auto position = static_cast<float>(i) * 0.01f;
            state.endEffectorTrail.push_back({ 0.7f + position, 0.1f, 0.3f });
        }

        return state;
    }

    class RobotArmSceneViewTest
        : public ::testing::Test
    {
    protected:
        RobotArmSceneViewTest()
        {
            ui::theme::SetCurrent(ui::theme::Instrument());
            view.SetState(ThreeJointArm(40), 2);
        }

        ui::backend::recording::RecordingCanvas canvas;
        RobotArmSceneView view;

        static constexpr ui::Rect bounds{ 0.0f, 0.0f, 1000.0f, 800.0f };
    };
}

TEST_F(RobotArmSceneViewTest, EachLinkDrawsItsTranslucentOutlineBeforeTheSolidStroke)
{
    view.Paint(canvas, bounds);

    std::vector<ui::Pen> linkPens;

    for (const auto& command : canvas.Commands())
        if (command.kind == CommandKind::SetPen && command.pen.cap == ui::LineCap::Round && command.pen.width >= 6.0f)
            linkPens.push_back(command.pen);

    ASSERT_EQ(linkPens.size(), 4u);

    for (std::size_t link = 0; link < 2; ++link)
    {
        const auto& outline = linkPens[link * 2];
        const auto& solid = linkPens[link * 2 + 1];

        EXPECT_GT(outline.width, solid.width);
        EXPECT_EQ(outline.color.alpha, 80);
        EXPECT_EQ(solid.color.alpha, 255);
        EXPECT_EQ(outline.color.WithAlpha(255), solid.color);
    }
}

TEST_F(RobotArmSceneViewTest, LinkColoursComeFromTheSeriesPalette)
{
    view.Paint(canvas, bounds);

    const auto& theme = ui::theme::Instrument();
    std::vector<ui::Color> solidLinks;

    for (const auto& command : canvas.Commands())
        if (command.kind == CommandKind::SetPen && command.pen.cap == ui::LineCap::Round && command.pen.color.alpha == 255 && command.pen.width >= 6.0f)
            solidLinks.push_back(command.pen.color);

    ASSERT_EQ(solidLinks.size(), 2u);
    EXPECT_EQ(solidLinks[0], theme.Series(0));
    EXPECT_EQ(solidLinks[1], theme.Series(1));
}

TEST_F(RobotArmSceneViewTest, TheTrailIsBandedRatherThanOneCallPerSegment)
{
    view.Paint(canvas, bounds);

    EXPECT_EQ(canvas.CountOf(CommandKind::DrawPolyline), 17u);
}

TEST_F(RobotArmSceneViewTest, TheTrailFadesTowardsTheNewestSample)
{
    view.Paint(canvas, bounds);

    std::vector<std::uint8_t> alphas;

    for (const auto& command : canvas.Commands())
        if (command.kind == CommandKind::SetPen && command.pen.width == 1.0f && command.pen.color.alpha > 0 && command.pen.color.alpha < 255)
            alphas.push_back(command.pen.color.alpha);

    ASSERT_GE(alphas.size(), 2u);
    EXPECT_LT(alphas.front(), alphas.back());
    EXPECT_LE(alphas.back(), 180);
}

TEST_F(RobotArmSceneViewTest, AShortTrailDrawsNoBands)
{
    view.SetState(ThreeJointArm(0), 2);
    canvas.Clear();
    view.Paint(canvas, bounds);

    EXPECT_EQ(canvas.CountOf(CommandKind::DrawPolyline), 1u);
}

TEST_F(RobotArmSceneViewTest, EveryJointGetsAMarkerAndTheBaseGetsAPlatform)
{
    view.Paint(canvas, bounds);

    EXPECT_EQ(canvas.CountOf(CommandKind::DrawEllipse), 3u);
    EXPECT_EQ(canvas.CountOf(CommandKind::DrawPolygon), 1u);
}

TEST_F(RobotArmSceneViewTest, TheOverlayKeepsItsMultiByteGlyphs)
{
    view.Paint(canvas, bounds);

    EXPECT_THAT(canvas.Texts(), ::testing::Contains("t = 1.25 s"));
    EXPECT_THAT(canvas.Texts(), ::testing::Contains("RNEA Inv. Dynamics:"));
    EXPECT_THAT(canvas.Texts(), ::testing::Contains(::testing::HasSubstr("τ")));
    EXPECT_THAT(canvas.Texts(), ::testing::Contains(::testing::HasSubstr("N·m")));
    EXPECT_THAT(canvas.Texts(), ::testing::Contains("EE: (0.700, 0.100, 0.300)"));
}

TEST_F(RobotArmSceneViewTest, TheBackgroundUsesTheSceneRole)
{
    view.Paint(canvas, bounds);

    ASSERT_FALSE(canvas.Commands().empty());
    EXPECT_EQ(canvas.Commands().front().kind, CommandKind::SetAntialiasing);
    EXPECT_EQ(canvas.Commands()[1].color,
        ui::theme::Instrument().Get(ui::theme::ColorRole::SceneBackground));
}

TEST_F(RobotArmSceneViewTest, PaintingWithNoRoomDrawsNothing)
{
    view.Paint(canvas, ui::Rect{});

    EXPECT_TRUE(canvas.Commands().empty());
}

TEST_F(RobotArmSceneViewTest, HoveringDoesNotOrbit)
{
    const auto before = view.Camera().Pose().azimuth;

    view.OnMouseMove(ui::MouseEvent{ ui::Point{ 100.0f, 100.0f }, ui::MouseButton::None, {} });
    view.OnMouseMove(ui::MouseEvent{ ui::Point{ 400.0f, 300.0f }, ui::MouseButton::None, {} });

    EXPECT_NEAR(view.Camera().Pose().azimuth, before, 1e-6f);
}

TEST_F(RobotArmSceneViewTest, DraggingOrbits)
{
    const auto before = view.Camera().Pose().azimuth;

    view.OnMousePress(ui::MouseEvent{ ui::Point{ 100.0f, 100.0f }, ui::MouseButton::Left, {} });
    view.OnMouseMove(ui::MouseEvent{ ui::Point{ 200.0f, 100.0f }, ui::MouseButton::None, {} });

    EXPECT_LT(view.Camera().Pose().azimuth, before);
}

TEST_F(RobotArmSceneViewTest, ReleasingStopsTheOrbit)
{
    view.OnMousePress(ui::MouseEvent{ ui::Point{ 100.0f, 100.0f }, ui::MouseButton::Left, {} });
    view.OnMouseRelease(ui::MouseEvent{ ui::Point{ 100.0f, 100.0f }, ui::MouseButton::Left, {} });

    const auto after = view.Camera().Pose().azimuth;
    view.OnMouseMove(ui::MouseEvent{ ui::Point{ 900.0f, 100.0f }, ui::MouseButton::None, {} });

    EXPECT_NEAR(view.Camera().Pose().azimuth, after, 1e-6f);
}

TEST_F(RobotArmSceneViewTest, ScrollingZoomsIn)
{
    const auto before = view.Camera().Pose().distance;

    view.OnWheel(ui::WheelEvent{ ui::Point{}, 120.0f, {} });

    EXPECT_NEAR(view.Camera().Pose().distance, before - 0.3f, 1e-5f);
}

TEST_F(RobotArmSceneViewTest, DoubleClickingResetsTheCamera)
{
    view.OnMousePress(ui::MouseEvent{ ui::Point{ 100.0f, 100.0f }, ui::MouseButton::Left, {} });
    view.OnMouseMove(ui::MouseEvent{ ui::Point{ 400.0f, 300.0f }, ui::MouseButton::None, {} });
    view.OnWheel(ui::WheelEvent{ ui::Point{}, 240.0f, {} });

    view.OnMouseDoubleClick(ui::MouseEvent{ ui::Point{}, ui::MouseButton::Left, {} });

    EXPECT_NEAR(view.Camera().Pose().azimuth, 0.8f, 1e-6f);
    EXPECT_NEAR(view.Camera().Pose().distance, 3.5f, 1e-6f);
}

TEST_F(RobotArmSceneViewTest, RepaintingProducesTheSameCommandStream)
{
    view.Paint(canvas, bounds);
    const auto first = canvas.Commands().size();

    canvas.Clear();
    view.Paint(canvas, bounds);

    EXPECT_EQ(canvas.Commands().size(), first);
}
