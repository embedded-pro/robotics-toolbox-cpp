#include "simulator/dynamics/RobotArm/view/RobotArmMainWindow.hpp"
#include <QSplitter>
#include <cmath>

namespace simulator::dynamics::view
{
    RobotArmMainWindow::RobotArmMainWindow(QWidget* parent)
        : QMainWindow(parent)
    {
        setWindowTitle("Robot Arm Dynamics Simulator");
        resize(1400, 900);

        auto* splitter = new QSplitter(Qt::Horizontal, this);

        configPanel = new RobotArmConfigurationPanel(splitter);
        configPanel->setMaximumWidth(350);
        configPanel->setMinimumWidth(280);

        view3D = new ui::backend::qt::QtPaintedWidget(sceneView, splitter);
        view3D->SetBackgroundRole(ui::theme::ColorRole::SceneBackground);
        view3D->SetPanCursorEnabled(true);

        splitter->addWidget(configPanel);
        splitter->addWidget(view3D);
        splitter->setStretchFactor(0, 0);
        splitter->setStretchFactor(1, 1);

        setCentralWidget(splitter);

        simulationTimer = new QTimer(this);
        connect(simulationTimer, &QTimer::timeout, this, &RobotArmMainWindow::OnSimulationStep);

        connect(configPanel, &RobotArmConfigurationPanel::StartRequested, this, &RobotArmMainWindow::OnStartRequested);
        connect(configPanel, &RobotArmConfigurationPanel::StopRequested, this, &RobotArmMainWindow::OnStopRequested);
        connect(configPanel, &RobotArmConfigurationPanel::ResetRequested, this, &RobotArmMainWindow::OnResetRequested);
        connect(configPanel, &RobotArmConfigurationPanel::ConfigurationChanged, this, &RobotArmMainWindow::ApplyConfiguration);

        ApplyConfiguration();
        statusBar()->showMessage("Configure robot parameters and press Start");
    }

    // QtPaintedWidget clears a callback on the view when it is destroyed, and QObject would
    // otherwise delete it during ~QMainWindow - after this window's own members, sceneView
    // among them, have already been destroyed. Deleting it here keeps that order valid.
    RobotArmMainWindow::~RobotArmMainWindow()
    {
        delete view3D;
    }

    void RobotArmMainWindow::ApplyConfiguration()
    {
        auto config = configPanel->GetConfiguration();
        simulator.Configure(config);
        simulator.SetInitialPositions(configPanel->GetInitialPositions());
        simulationTimer->setInterval(static_cast<int>(std::round(1000.0f * config.dt)));
        sceneView.SetState(simulator.GetState(), config.dof);
    }

    void RobotArmMainWindow::OnStartRequested()
    {
        if (!running)
        {
            ApplyConfiguration();
            simulator.SetInitialPositions(configPanel->GetInitialPositions());
            running = true;
            simulationTimer->start();
            statusBar()->showMessage("Simulation running...");
        }
    }

    void RobotArmMainWindow::OnStopRequested()
    {
        running = false;
        simulationTimer->stop();
        statusBar()->showMessage("Simulation stopped");
    }

    void RobotArmMainWindow::OnResetRequested()
    {
        running = false;
        simulationTimer->stop();
        ApplyConfiguration();
        statusBar()->showMessage("Simulation reset");
    }

    void RobotArmMainWindow::OnSimulationStep()
    {
        simulator.SetTorques(configPanel->GetTorques());
        simulator.Step();

        auto config = simulator.GetConfig();
        sceneView.SetState(simulator.GetState(), config.dof);

        auto& state = simulator.GetState();
        statusBar()->showMessage(
            QString("t=%1s | Running")
                .arg(static_cast<double>(state.time), 0, 'f', 2));
    }
}
