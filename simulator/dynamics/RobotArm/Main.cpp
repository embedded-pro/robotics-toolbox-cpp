#include "simulator/dynamics/RobotArm/view/RobotArmMainWindow.hpp"
#include "simulator/shell/AppRunner.hpp"

int main(int argc, char* argv[])
{
    return simulator::shell::Run<simulator::dynamics::view::RobotArmMainWindow>(argc, argv, ui::theme::Instrument());
}
