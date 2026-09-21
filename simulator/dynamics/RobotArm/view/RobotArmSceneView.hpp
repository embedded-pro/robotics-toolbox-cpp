#pragma once

#include "simulator/dynamics/RobotArm/application/RobotArmTypes.hpp"
#include "ui/core/PaintedView.hpp"
#include "ui/scene/Camera3D.hpp"
#include "ui/scene/SceneGizmos.hpp"
#include <array>
#include <cstdint>
#include <vector>

namespace simulator::dynamics::view
{
    struct RobotArmSceneConfig
    {
        float linkWidth{ 6.0f };
        float linkOutlineWidth{ 8.0f };
        std::uint8_t linkOutlineAlpha{ 80 };

        float shadowWidth{ 3.0f };
        std::uint8_t shadowAlpha{ 80 };

        float baseRadius{ 10.0f };
        float endEffectorRadius{ 8.0f };
        float jointRadius{ 7.0f };
        float platformHalfExtent{ 0.1f };
        std::uint8_t platformAlpha{ 150 };

        float crosshairInner{ 10.0f };
        float crosshairOuter{ 14.0f };
        std::uint8_t crosshairAlpha{ 150 };

        std::uint8_t trailMaximumAlpha{ 180 };

        std::size_t trailBands{ 16 };
        std::size_t trailSeriesIndex{ 3 };

        float overlayLeft{ 10.0f };
        float overlayFirstBaseline{ 20.0f };
        float overlayLineHeight{ 18.0f };
        float overlaySectionGap{ 4.0f };
        float hintBottomOffset{ 10.0f };
    };

    class RobotArmSceneView
        : public ui::PaintedView
    {
    public:
        explicit RobotArmSceneView(RobotArmSceneConfig config = {});

        void SetState(const RobotArmState& state, int dof);

        void Paint(ui::Canvas& canvas, const ui::Rect& bounds) override;
        [[nodiscard]] ui::Size MinimumSize() const override;

        void OnMousePress(const ui::MouseEvent& event) override;
        void OnMouseMove(const ui::MouseEvent& event) override;
        void OnMouseRelease(const ui::MouseEvent& event) override;
        void OnMouseDoubleClick(const ui::MouseEvent& event) override;
        void OnWheel(const ui::WheelEvent& event) override;

        [[nodiscard]] const ui::scene::OrbitCamera& Camera() const;

    private:
        void ProjectGeometry(const ui::scene::ViewFrame& frame);
        void DrawShadow(ui::Canvas& canvas) const;
        void DrawTrail(ui::Canvas& canvas) const;
        void DrawLinks(ui::Canvas& canvas) const;
        void DrawJoints(ui::Canvas& canvas, const ui::scene::ViewFrame& frame);
        void DrawInfoOverlay(ui::Canvas& canvas, const ui::Rect& bounds);

        RobotArmSceneConfig config;
        ui::scene::OrbitCamera camera;

        RobotArmState currentState;
        int currentDof{ 2 };

        std::vector<ui::Point> jointScreen;
        std::vector<ui::Point> shadowScreen;
        std::vector<ui::Point> trailScreen;
        std::vector<ui::Point> bandScratch;
        std::array<ui::Point, 4> platformScratch{};
        std::array<char, 96> textScratch{};
    };
}
