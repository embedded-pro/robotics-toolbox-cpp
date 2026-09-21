#include "simulator/dynamics/RobotArm/view/RobotArmMainWindow.hpp"
#include "ui/theme/Theme.hpp"
#include <QApplication>

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    ui::theme::SetCurrent(ui::theme::Instrument());

    simulator::dynamics::view::RobotArmMainWindow window;
    window.show();

    return app.exec();
}
