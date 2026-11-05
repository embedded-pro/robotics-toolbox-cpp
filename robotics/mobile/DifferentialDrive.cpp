#include "robotics/mobile/DifferentialDrive.hpp"

namespace mobile
{
    template class DifferentialDrive<float>;
    template WheelSpeeds<float> ScaleToLimit<float>(const WheelSpeeds<float>&, float);
}
