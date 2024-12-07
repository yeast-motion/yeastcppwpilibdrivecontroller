#pragma once

#include <iostream>
#include <vector>

#include <yeastcpp.hpp>

#include <wpimath/frc/kinematics/SwerveDriveKinematics.h>

namespace yeast_motion
{
    class WPILibDriveController : DriveController
    {
        public:
            MotionState drive(MotionCommand command);
            WPILibDriveController(nlohmann::json characterization);

        private:
            std::vector<SwerveModule> modules;
            std::unique_ptr<frc::SwerveDriveKinematics<4>> kinematics;
    };
}