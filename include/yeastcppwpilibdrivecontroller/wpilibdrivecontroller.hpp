#pragma once

#include <iostream>

#include <yeastcpp.hpp>

namespace yeast_motion
{
    class WPILibDriveController : DriveController
    {
        MotionState drive(MotionCommand command);
    };
}