#pragma once

#include "simulator/dynamics/RobotArm/application/RobotArmSimulator.hpp"
#include "simulator/dynamics/RobotArm/view/RobotArmConfigurationPanel.hpp"
#include "simulator/dynamics/RobotArm/view/RobotArmSceneView.hpp"
#include "ui/backend/qt/QtPaintedWidget.hpp"
#include <QMainWindow>
#include <QStatusBar>
#include <QTimer>

namespace simulator::dynamics::view
{
    class RobotArmMainWindow
        : public QMainWindow
    {
        Q_OBJECT

    public:
        explicit RobotArmMainWindow(QWidget* parent = nullptr);
        ~RobotArmMainWindow() override;

    private:
        void OnStartRequested();
        void OnStopRequested();
        void OnResetRequested();
        void OnSimulationStep();
        void ApplyConfiguration();

        RobotArmSimulator simulator;
        RobotArmSceneView sceneView;
        RobotArmConfigurationPanel* configPanel;
        ui::backend::qt::QtPaintedWidget* view3D;
        QTimer* simulationTimer;
        bool running = false;
    };
}
