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

    if (modules.size() != 4)
    {
        throw std::runtime_error("Only configurations with 4 swerve modules are supported by the Yeast WPI Lib Drive Controller");
    }

    frc::Translation2d m_frontLeftLocation  {units::meter_t(modules[0].translation.x), units::meter_t(modules[0].translation.y)};
    frc::Translation2d m_frontRightLocation {units::meter_t(modules[1].translation.x), units::meter_t(modules[1].translation.y)};
    frc::Translation2d m_backLeftLocation   {units::meter_t(modules[2].translation.x), units::meter_t(modules[2].translation.y)};
    frc::Translation2d m_backRightLocation  {units::meter_t(modules[3].translation.x), units::meter_t(modules[3].translation.y)};

    kinematics.reset(new frc::SwerveDriveKinematics<4> 
        (m_frontLeftLocation, 
        m_frontRightLocation, 
        m_backLeftLocation,
        m_backRightLocation));
}

MotionState WPILibDriveController::drive(MotionCommand command)
{
    MotionState result;

    frc::ChassisSpeeds speeds
        (units::meters_per_second_t(command.velocity.x), 
         units::meters_per_second_t(command.velocity.y), 
         units::radians_per_second_t(command.velocity.omega));
         
    auto [fl, fr, bl, br] = kinematics->ToSwerveModuleStates(speeds);

    auto flOptimized = frc::SwerveModuleState::Optimize(fl, units::radian_t(m_turningEncoder.GetDistance()));

    
}