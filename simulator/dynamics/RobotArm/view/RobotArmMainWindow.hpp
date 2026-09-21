#pragma once

#include "simulator/dynamics/RobotArm/application/RobotArmForm.hpp"
#include "simulator/dynamics/RobotArm/application/RobotArmSimulator.hpp"
#include "simulator/dynamics/RobotArm/view/RobotArmSceneView.hpp"
#include "ui/backend/qt/QtAppShell.hpp"
#include "ui/backend/qt/QtFormView.hpp"
#include "ui/backend/qt/QtPaintedWidget.hpp"
#include <QMainWindow>
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
        void OnActionTriggered(ui::model::ActionId action);
        void OnStartRequested();
        void OnStopRequested();
        void OnResetRequested();
        void OnSimulationStep();
        void ApplyConfiguration();

        RobotArmSimulator simulator;
        RobotArmForm form;
        RobotArmSceneView sceneView;
        ui::backend::qt::QtFormView* formView;
        ui::backend::qt::QtAppShell shell;
        ui::backend::qt::QtPaintedWidget* view3D;
        QTimer* simulationTimer;
        bool running = false;
    };
}
