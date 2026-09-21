#include "simulator/dynamics/RobotArm/view/RobotArmMainWindow.hpp"
#include <cmath>

namespace simulator::dynamics::view
{
    namespace
    {
        const ui::shell::ShellSpec shellSpec{
            "Robot Arm Dynamics Simulator",
            ui::Size{ 1400.0f, 900.0f },
            350.0f,
            {},
            "Configure robot parameters and press Start"
        };
    }

    RobotArmMainWindow::RobotArmMainWindow(QWidget* parent)
        : QMainWindow(parent)
        , formView(new ui::backend::qt::QtFormView{ this })
        , shell(*this, shellSpec)
        , view3D(new ui::backend::qt::QtPaintedWidget{ sceneView, this })
        , simulationTimer(new QTimer{ this })
    {
        formView->Build(form.Model());
        formView->setMinimumWidth(280);
        shell.SetPanel(formView);

        view3D->SetBackgroundRole(ui::theme::ColorRole::SceneBackground);
        view3D->SetPanCursorEnabled(true);
        shell.SetContent(view3D);

        connect(simulationTimer, &QTimer::timeout, this, &RobotArmMainWindow::OnSimulationStep);

        form.Model().onActionTriggered = [this](ui::model::ActionId action)
        {
            OnActionTriggered(action);
        };

        ApplyConfiguration();
    }

    RobotArmMainWindow::~RobotArmMainWindow()
    {
        delete view3D;
    }

    void RobotArmMainWindow::OnActionTriggered(ui::model::ActionId action)
    {
        if (action == field::start)
            OnStartRequested();
        else if (action == field::stop)
            OnStopRequested();
        else if (action == field::reset)
            OnResetRequested();
    }

    void RobotArmMainWindow::ApplyConfiguration()
    {
        auto config = form.BuildConfiguration();
        simulator.Configure(config);
        simulator.SetInitialPositions(form.InitialPositions());
        simulationTimer->setInterval(static_cast<int>(std::round(1000.0f * config.dt)));
        sceneView.SetState(simulator.GetState(), config.dof);
    }

    void RobotArmMainWindow::OnStartRequested()
    {
        if (!running)
        {
            ApplyConfiguration();
            simulator.SetInitialPositions(form.InitialPositions());
            running = true;
            simulationTimer->start();
            shell.SetStatus("Simulation running...");
        }
    }

    void RobotArmMainWindow::OnStopRequested()
    {
        running = false;
        simulationTimer->stop();
        shell.SetStatus("Simulation stopped");
    }

    void RobotArmMainWindow::OnResetRequested()
    {
        running = false;
        simulationTimer->stop();
        ApplyConfiguration();
        shell.SetStatus("Simulation reset");
    }

    void RobotArmMainWindow::OnSimulationStep()
    {
        simulator.SetTorques(form.Torques());
        simulator.Step();

        auto config = simulator.GetConfig();
        sceneView.SetState(simulator.GetState(), config.dof);

        auto& state = simulator.GetState();

        shell.SetStatus(QString("t=%1s | Running")
                .arg(static_cast<double>(state.time), 0, 'f', 2)
                .toStdString());
    }
}
