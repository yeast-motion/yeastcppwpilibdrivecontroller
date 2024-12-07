#include <exception>

#include "wpilibdrivecontroller.hpp"

#include "wpimath/frc/estimator/SwerveDrivePoseEstimator.h"
#include "wpimath/frc/kinematics/SwerveDriveKinematics.h"
#include "wpimath/frc/kinematics/SwerveDriveOdometry.h"
#include "wpimath/frc/kinematics/SwerveModulePosition.h"
#include "wpimath/frc/kinematics/SwerveModuleState.h"
#include "wpimath/frc/kinematics/ChassisSpeeds.h"
#include "wpimath/frc/geometry/Rotation2d.h"
#include "wpimath/frc/geometry/Translation2d.h"
#include "wpimath/frc/geometry/Pose2d.h"

using namespace yeast_motion;


WPILibDriveController::WPILibDriveController(nlohmann::json characterization)
{
    // Need to load characterizations, incl swerve modules

    if (module_statuses.size() != 4)
    {
        throw std::runtime_error("Only configurations with 4 swerve modules are supported by the Yeast WPI Lib Drive Controller");
    }

    frc::Translation2d m_frontLeftLocation  {units::meter_t(module_configs[0].translation.x), units::meter_t(module_configs[0].translation.y)};
    frc::Translation2d m_frontRightLocation {units::meter_t(module_configs[1].translation.x), units::meter_t(module_configs[1].translation.y)};
    frc::Translation2d m_backLeftLocation   {units::meter_t(module_configs[2].translation.x), units::meter_t(module_configs[2].translation.y)};
    frc::Translation2d m_backRightLocation  {units::meter_t(module_configs[3].translation.x), units::meter_t(module_configs[3].translation.y)};

    // https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#constructing-the-kinematics-object
    kinematics.reset(new frc::SwerveDriveKinematics<4> 
        (m_frontLeftLocation, 
        m_frontRightLocation, 
        m_backLeftLocation,
        m_backRightLocation));

    for (size_t i = 0; i < 4; i++)
    {
        SwerveModuleCommand command;
        module_commands.push_back(command);
    }
}

MotionState WPILibDriveController::drive(MotionCommand command)
{
    MotionState result;

    // https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#converting-chassis-speeds-to-module-states
    frc::ChassisSpeeds speeds
        (units::meters_per_second_t(command.velocity.x), 
         units::meters_per_second_t(command.velocity.y), 
         units::radians_per_second_t(command.velocity.omega));
         
    auto [fl, fr, bl, br] = kinematics->ToSwerveModuleStates(speeds);
    
    std::vector<frc::SwerveModuleState> optimized_modules;

    // https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#module-angle-optimization
    optimized_modules.push_back(frc::SwerveModuleState::Optimize(fl, units::radian_t(module_statuses[0].theta)));
    optimized_modules.push_back(frc::SwerveModuleState::Optimize(fr, units::radian_t(module_statuses[1].theta)));
    optimized_modules.push_back(frc::SwerveModuleState::Optimize(bl, units::radian_t(module_statuses[2].theta)));
    optimized_modules.push_back(frc::SwerveModuleState::Optimize(br, units::radian_t(module_statuses[3].theta)));

    // https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#cosine-compensation
    optimized_modules[0].CosineScale(units::radian_t(module_statuses[0].theta));
    optimized_modules[1].CosineScale(units::radian_t(module_statuses[1].theta));
    optimized_modules[2].CosineScale(units::radian_t(module_statuses[2].theta));
    optimized_modules[3].CosineScale(units::radian_t(module_statuses[3].theta));

    module_commands[0].speed = optimized_modules[0].speed.value();
    module_commands[0].theta = optimized_modules[0].angle.Radians().value();
    module_commands[1].speed = optimized_modules[1].speed.value();
    module_commands[1].theta = optimized_modules[1].angle.Radians().value();
    module_commands[2].speed = optimized_modules[2].speed.value();
    module_commands[2].theta = optimized_modules[2].angle.Radians().value();
    module_commands[3].speed = optimized_modules[3].speed.value();
    module_commands[3].theta = optimized_modules[3].angle.Radians().value();
    
}