#pragma once

#include <iostream>
#include <vector>
#include <any>

#include "yeastcpp/data_structures/swerve_module_command.hpp"
#include "yeastcpp/data_structures/swerve_module_config.hpp"
#include "yeastcpp/data_structures/swerve_module_status.hpp"

#include "yeastcpp/components/drive_controller.hpp"

namespace frc
{
    template <size_t NumModules>
    class SwerveDriveKinematics;
}

namespace yeast_motion
{
    class WPILibDriveController : public DriveController
    {
        public:
            MotionSample drive(MotionCommand command);
            std::vector<SwerveModuleCommand> get_command();
            void update_motor_status (std::vector<SwerveModuleStatus> motor_status);
            WPILibDriveController(nlohmann::json characterization);

        private:
            std::vector<SwerveModuleConfig> module_configs;
            std::vector<SwerveModuleStatus> module_statuses;
            std::vector<SwerveModuleCommand> module_commands;
            std::unique_ptr<frc::SwerveDriveKinematics<4>> kinematics;
        };
}