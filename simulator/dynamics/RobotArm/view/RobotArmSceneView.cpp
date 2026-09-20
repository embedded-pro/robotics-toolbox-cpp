#include "simulator/dynamics/RobotArm/view/RobotArmSceneView.hpp"
#include "ui/core/Format.hpp"
#include "ui/theme/Theme.hpp"
#include <algorithm>
#include <numbers>

namespace simulator::dynamics::view
{
    namespace
    {
        using ui::scene::Vector3;

        [[nodiscard]] Vector3 ToVector(const std::array<float, 3>& point)
        {
            return Vector3{ point[0], point[1], point[2] };
        }

        [[nodiscard]] Vector3 Flattened(const std::array<float, 3>& point)
        {
            return Vector3{ point[0], point[1], 0.0f };
        }
    }

    RobotArmSceneView::RobotArmSceneView(RobotArmSceneConfig config)
        : config(config)
    {}

    void RobotArmSceneView::SetState(const RobotArmState& state, int dof)
    {
        currentState = state;
        currentDof = dof;

        // Reserved here rather than in Paint, which must not allocate.
        jointScreen.reserve(currentState.jointPositions.size());
        shadowScreen.reserve(currentState.jointPositions.size());
        trailScreen.reserve(currentState.endEffectorTrail.size());
        bandScratch.reserve(currentState.endEffectorTrail.size());

        RequestRepaint();
    }

    ui::Size RobotArmSceneView::MinimumSize() const
    {
        return ui::Size{ 400.0f, 400.0f };
    }

    const ui::scene::OrbitCamera& RobotArmSceneView::Camera() const
    {
        return camera;
    }

    void RobotArmSceneView::OnMousePress(const ui::MouseEvent& event)
    {
        if (event.button == ui::MouseButton::Left)
            camera.StartOrbit(event.position);
    }

    // No button test here: the Qt adapter reports MouseButton::None on every move. The camera's
    // own orbiting flag is what distinguishes a drag from a hover.
    void RobotArmSceneView::OnMouseMove(const ui::MouseEvent& event)
    {
        if (!camera.IsOrbiting())
            return;

        camera.UpdateOrbit(event.position);
        RequestRepaint();
    }

    void RobotArmSceneView::OnMouseRelease(const ui::MouseEvent& event)
    {
        static_cast<void>(event);
        camera.EndOrbit();
    }

    void RobotArmSceneView::OnMouseDoubleClick(const ui::MouseEvent& event)
    {
        static_cast<void>(event);
        camera.Reset();
        RequestRepaint();
    }

    void RobotArmSceneView::OnWheel(const ui::WheelEvent& event)
    {
        camera.Zoom(event.delta);
        RequestRepaint();
    }

    void RobotArmSceneView::Paint(ui::Canvas& canvas, const ui::Rect& bounds)
    {
        if (bounds.IsEmpty())
            return;

        canvas.SetAntialiasing(true);
        canvas.FillRect(bounds, ui::theme::Current().Get(ui::theme::ColorRole::SceneBackground));

        const auto frame = camera.FrameFor(bounds);
        ProjectGeometry(frame);

        ui::scene::DrawGroundGrid(canvas, frame);
        ui::scene::DrawAxisTriad(canvas, frame);

        DrawShadow(canvas);
        DrawTrail(canvas);
        DrawLinks(canvas);
        DrawJoints(canvas, frame);
        DrawInfoOverlay(canvas, bounds);
    }

    void RobotArmSceneView::ProjectGeometry(const ui::scene::ViewFrame& frame)
    {
        jointScreen.clear();
        shadowScreen.clear();
        trailScreen.clear();

        for (const auto& joint : currentState.jointPositions)
        {
            jointScreen.push_back(frame.Project(ToVector(joint)));
            shadowScreen.push_back(frame.Project(Flattened(joint)));
        }

        for (const auto& point : currentState.endEffectorTrail)
            trailScreen.push_back(frame.Project(ToVector(point)));
    }

    void RobotArmSceneView::DrawShadow(ui::Canvas& canvas) const
    {
        if (shadowScreen.size() < 2)
            return;

        const auto shadow = ui::theme::Current().Get(ui::theme::ColorRole::Neutral).WithAlpha(config.shadowAlpha);

        canvas.SetPen(ui::Pen{ shadow, config.shadowWidth, ui::LineStyle::Solid, ui::LineCap::Round });
        canvas.DrawPolyline(shadowScreen);
    }

    void RobotArmSceneView::DrawTrail(ui::Canvas& canvas) const
    {
        if (trailScreen.size() < 2 || config.trailBands == 0)
            return;

        const auto base = ui::theme::Current().Series(config.trailSeriesIndex);
        const auto total = trailScreen.size();
        const auto bands = std::min(config.trailBands, total - 1);

        for (std::size_t band = 0; band < bands; ++band)
        {
            const auto first = total * band / bands;
            const auto last = total * (band + 1) / bands - 1;

            if (last <= first)
                continue;

            // Bands share an endpoint so the polyline has no gaps at the seams, and the alpha is
            // the original's ramp sampled at the band's newest sample.
            const auto span = std::span<const ui::Point>{ trailScreen }.subspan(first, last - first + 1);
            const auto ratio = static_cast<float>(last + 1) / static_cast<float>(total);
            const auto alpha = static_cast<std::uint8_t>(ratio * static_cast<float>(config.trailMaximumAlpha));

            canvas.SetPen(ui::Pen{ base.WithAlpha(alpha) });
            canvas.DrawPolyline(span);
        }
    }

    void RobotArmSceneView::DrawLinks(ui::Canvas& canvas) const
    {
        if (jointScreen.size() < 2)
            return;

        const auto& theme = ui::theme::Current();

        for (std::size_t i = 0; i + 1 < jointScreen.size(); ++i)
        {
            const auto color = theme.Series(i);

            // Wide translucent stroke first, solid over it. The Qt original drew these the other
            // way round, so the outline it intended was painted on top of the link and never read
            // as an outline at all.
            canvas.SetPen(ui::Pen{ color.WithAlpha(config.linkOutlineAlpha), config.linkOutlineWidth,
                ui::LineStyle::Solid, ui::LineCap::Round });
            canvas.DrawLine(jointScreen[i], jointScreen[i + 1]);

            canvas.SetPen(ui::Pen{ color, config.linkWidth, ui::LineStyle::Solid, ui::LineCap::Round });
            canvas.DrawLine(jointScreen[i], jointScreen[i + 1]);
        }
    }

    void RobotArmSceneView::DrawJoints(ui::Canvas& canvas, const ui::scene::ViewFrame& frame)
    {
        const auto& theme = ui::theme::Current();

        for (std::size_t i = 0; i < jointScreen.size(); ++i)
        {
            const auto position = jointScreen[i];
            const auto isBase = i == 0;
            const auto isEndEffector = i + 1 == jointScreen.size();

            if (isBase)
            {
                canvas.SetPen(ui::Pen{ theme.Get(ui::theme::ColorRole::Text), 2.0f });
                canvas.SetBrush(ui::Brush{ theme.Get(ui::theme::ColorRole::Neutral) });
                canvas.DrawEllipse(position, config.baseRadius, config.baseRadius);

                const auto extent = config.platformHalfExtent;
                platformScratch[0] = frame.Project(Vector3{ -extent, -extent, 0.0f });
                platformScratch[1] = frame.Project(Vector3{ extent, -extent, 0.0f });
                platformScratch[2] = frame.Project(Vector3{ extent, extent, 0.0f });
                platformScratch[3] = frame.Project(Vector3{ -extent, extent, 0.0f });

                canvas.SetPen(ui::Pen{ theme.Get(ui::theme::ColorRole::GridMajor) });
                canvas.SetBrush(ui::Brush{ theme.Get(ui::theme::ColorRole::Surface).WithAlpha(config.platformAlpha) });
                canvas.DrawPolygon(platformScratch);
            }
            else if (isEndEffector)
            {
                canvas.SetPen(ui::Pen{ theme.Get(ui::theme::ColorRole::Text), 2.0f });
                canvas.SetBrush(ui::Brush{ theme.Series(config.trailSeriesIndex) });
                canvas.DrawEllipse(position, config.endEffectorRadius, config.endEffectorRadius);

                canvas.SetPen(ui::Pen{ theme.Get(ui::theme::ColorRole::Crosshair).WithAlpha(config.crosshairAlpha) });
                canvas.DrawLine(ui::Point{ position.x - config.crosshairOuter, position.y },
                    ui::Point{ position.x - config.crosshairInner, position.y });
                canvas.DrawLine(ui::Point{ position.x + config.crosshairInner, position.y },
                    ui::Point{ position.x + config.crosshairOuter, position.y });
                canvas.DrawLine(ui::Point{ position.x, position.y - config.crosshairOuter },
                    ui::Point{ position.x, position.y - config.crosshairInner });
                canvas.DrawLine(ui::Point{ position.x, position.y + config.crosshairInner },
                    ui::Point{ position.x, position.y + config.crosshairOuter });
            }
            else
            {
                canvas.SetPen(ui::Pen{ theme.Get(ui::theme::ColorRole::Text), 2.0f });
                canvas.SetBrush(ui::Brush{ theme.Get(ui::theme::ColorRole::Surface) });
                canvas.DrawEllipse(position, config.jointRadius, config.jointRadius);
            }
        }

        canvas.SetBrush(ui::Brush{});
    }

    void RobotArmSceneView::DrawInfoOverlay(ui::Canvas& canvas, const ui::Rect& bounds)
    {
        const auto& theme = ui::theme::Current();
        const auto left = bounds.Left() + config.overlayLeft;
        auto baseline = bounds.Top() + config.overlayFirstBaseline;

        canvas.SetFont(theme.Get(ui::theme::FontRole::Monospace));
        canvas.SetPen(ui::Pen{ theme.Get(ui::theme::ColorRole::Text) });

        auto written = ui::FormatInto(textScratch, "t = {:.2f} s", currentState.time);
        canvas.DrawText(ui::Point{ left, baseline }, std::string_view{ textScratch.data(), written });
        baseline += config.overlayLineHeight;

        const auto jointsShown = std::min(static_cast<std::size_t>(std::max(currentDof, 0)), currentState.q.size());

        for (std::size_t i = 0; i < jointsShown; ++i)
        {
            const auto degrees = currentState.q[i] * 180.0f / std::numbers::pi_v<float>;
            const auto rate = i < currentState.qDot.size() ? currentState.qDot[i] : 0.0f;

            written = ui::FormatInto(textScratch, "q{} = {:7.1f} deg  |  dq = {:7.2f} rad/s", i + 1, degrees, rate);
            canvas.DrawText(ui::Point{ left, baseline }, std::string_view{ textScratch.data(), written });
            baseline += config.overlayLineHeight;
        }

        if (!currentState.inverseDynamicsTorques.empty())
        {
            baseline += config.overlaySectionGap;

            canvas.SetPen(ui::Pen{ theme.Get(ui::theme::ColorRole::Warning) });
            canvas.DrawText(ui::Point{ left, baseline }, "RNEA Inv. Dynamics:");
            baseline += config.overlayLineHeight;

            const auto torquesShown = std::min(static_cast<std::size_t>(std::max(currentDof, 0)),
                currentState.inverseDynamicsTorques.size());

            for (std::size_t i = 0; i < torquesShown; ++i)
            {
                written = ui::FormatInto(textScratch, "  τ{} = {:8.3f} N·m", i + 1,
                    currentState.inverseDynamicsTorques[i]);
                canvas.DrawText(ui::Point{ left, baseline }, std::string_view{ textScratch.data(), written });
                baseline += config.overlayLineHeight;
            }
        }

        canvas.SetPen(ui::Pen{ theme.Get(ui::theme::ColorRole::Text) });

        if (!currentState.jointPositions.empty())
        {
            const auto& endEffector = currentState.jointPositions.back();

            written = ui::FormatInto(textScratch, "EE: ({:.3f}, {:.3f}, {:.3f})",
                endEffector[0], endEffector[1], endEffector[2]);
            canvas.DrawText(ui::Point{ left, baseline }, std::string_view{ textScratch.data(), written });
        }

        canvas.SetFont(theme.Get(ui::theme::FontRole::Small));
        canvas.SetPen(ui::Pen{ theme.Get(ui::theme::ColorRole::TextMuted) });
        canvas.DrawText(ui::Point{ left, bounds.Bottom() - config.hintBottomOffset }, "LMB: Rotate  |  Scroll: Zoom");
    }
}
