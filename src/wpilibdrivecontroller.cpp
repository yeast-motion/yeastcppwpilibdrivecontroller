#include <exception>

#include "yeastcppwpilibdrivecontroller/wpilibdrivecontroller.hpp"

#include "frc/estimator/SwerveDrivePoseEstimator.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "frc/kinematics/SwerveDriveOdometry.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Translation2d.h"
#include "frc/geometry/Pose2d.h"

using namespace yeast_motion;


WPILibDriveController::WPILibDriveController(nlohmann::json characterization)
{
    for (auto motor_config : characterization["MotorConfig"])
    {
        SwerveModuleConfig config;
        config.translation.x = motor_config["x"];
        config.translation.y = motor_config["y"];
        module_configs.push_back(config);
    }
    
    if (module_configs.size() != 4)
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

MotionSample WPILibDriveController::drive(MotionCommand command)
{
    // https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#converting-chassis-speeds-to-module-states
    frc::ChassisSpeeds speeds
        (units::meters_per_second_t(command.velocity.x), 
         units::meters_per_second_t(command.velocity.y), 
         units::radians_per_second_t(command.velocity.omega));
         
    auto [fl, fr, bl, br] = kinematics->ToSwerveModuleStates(speeds);

    if (std::abs(speeds.vx.value()) <= 0.0001 &&
        std::abs(speeds.vy.value()) <= 0.0001 &&
        std::abs(speeds.omega.value()) <= 0.0001)
    {
        fl.speed = 0_mps;
        fl.angle = units::radian_t(module_statuses[0].theta);
        fr.speed = 0_mps;
        fr.angle = units::radian_t(module_statuses[1].theta);
        bl.speed = 0_mps;
        bl.angle = units::radian_t(module_statuses[2].theta);
        br.speed = 0_mps;
        br.angle = units::radian_t(module_statuses[3].theta);
    }

    std::vector<frc::SwerveModuleState> optimized_modules;

    // https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#module-angle-optimization
    fl.Optimize(frc::Rotation2d(units::radian_t(module_statuses[0].theta)));
    fr.Optimize(frc::Rotation2d(units::radian_t(module_statuses[1].theta)));
    bl.Optimize(frc::Rotation2d(units::radian_t(module_statuses[2].theta)));
    br.Optimize(frc::Rotation2d(units::radian_t(module_statuses[3].theta)));

    optimized_modules.push_back(fl);
    optimized_modules.push_back(fr);
    optimized_modules.push_back(bl);
    optimized_modules.push_back(br);

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

    MotionSample motion_ref;
    motion_ref.pose_valid = false;
    motion_ref.velocity = command.velocity;
    motion_ref.velocity_valid = command.velocity_valid;
    motion_ref.acceleration = command.acceleration;
    motion_ref.acceleration_valid = command.acceleration_valid;
    
    return motion_ref;
}


std::vector<SwerveModuleCommand> WPILibDriveController::get_command()
{
    return module_commands;
}


void WPILibDriveController::update_motor_status (std::vector<SwerveModuleStatus> motor_status)
{
    module_statuses = motor_status;
}