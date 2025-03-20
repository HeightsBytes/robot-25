#pragma once

#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <pathplanner/lib/path/PathConstraints.h>
//#include <pathplanner/lib/config/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/config/PIDConstants.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

namespace DriveConstants {
    namespace CanIds{
        inline constexpr int kFrontLeftDriveMotorPort = 4; // placeholder
        inline constexpr int kRearLeftDriveMotorPort = 2;
        inline constexpr int kFrontRightDriveMotorPort = 6;
        inline constexpr int kRearRightDriveMotorPort = 8;

        inline constexpr int kFrontLeftTurningMotorPort = 3;
        inline constexpr int kRearLeftTurningMotorPort = 1;
        inline constexpr int kFrontRightTurningMotorPort = 5;
        inline constexpr int kRearRightTurningMotorPort = 7;

        inline constexpr int kFrontLeftTurningEncoderPorts = 1;
        inline constexpr int kRearLeftTurningEncoderPorts = 2;
        inline constexpr int kFrontRightTurningEncoderPorts = 3;
        inline constexpr int kRearRightTurningEncoderPorts = 4;

        inline constexpr int kPidgeonID = 0;
    } // namespace CanIds
  inline constexpr double kFrontRightOffset = 104.77 - 180;
  inline constexpr double kFrontLeftOffset = -133.95;
  inline constexpr double kRearRightOffset = 15.03;
  inline constexpr double kRearLeftOffset = -56.43 - 180;

  inline constexpr bool kFrontRightInverted = true;
  inline constexpr bool kFrontLeftInverted = true;
  inline constexpr bool kRearRightInverted = true;
  inline constexpr bool kRearLeftInverted = true;

  inline constexpr auto kMaxChassisSpeed = 4.25_mps;
  inline constexpr auto kMaxAngularSpeed =
    units::radians_per_second_t(1 * std::numbers::pi);
  inline constexpr auto kMaxAngularAcceleration =
    units::radians_per_second_squared_t(2 * std::numbers::pi);

  inline constexpr auto kTrackWidth = 0.31369_m;
  inline constexpr auto kTrackLength = 0.31369_m;

  inline frc::SwerveDriveKinematics<4> kDriveKinematics{ // ++, +-, -+, --
    frc::Translation2d(kTrackLength, kTrackWidth),
    frc::Translation2d(kTrackLength, -kTrackWidth),
    frc::Translation2d(-kTrackLength, kTrackWidth),
    frc::Translation2d(-kTrackLength, -kTrackWidth)};
} // namespace DriveConstants

namespace ModuleConstants{
  inline constexpr double kGearRatio = 1 / 6.75;
  inline constexpr double kWheelDiameterMeters = 0.05092958;
  inline constexpr double kDriveEncoderDistancePerPulse =
    kGearRatio * 2 * std::numbers::pi * kWheelDiameterMeters;

  inline constexpr double kTurnRatio = 7.0 / 150.0;
  inline constexpr double kTurnEncoderRatio = kTurnRatio * 2.0 * std::numbers::pi;

  inline constexpr double kPDrive = 0.175;
  inline constexpr double kIDrive = 0;
  inline constexpr double kDDrive = 0.02;
  inline constexpr double kFFDrive = 2.67;

  inline constexpr double kPTurn = 1.25;
  inline constexpr double kITurn = 0;
  inline constexpr double kDTurn = 0;
  inline constexpr double kFFTurn = 0;

  inline constexpr auto kMaxModuleSpeed = 4.5_mps;
} // namespace ModuleConstants

namespace VisionConstants {
inline const frc::Transform3d RightTransform{
    frc::Translation3d(-15_in, -7_in, 24_in),
    frc::Rotation3d{0_deg, 0_deg, -150_deg}};
inline const frc::Transform3d LeftTransform{
    frc::Translation3d(-15_in, 7_in, 24_in),
    frc::Rotation3d{0_deg, 0_deg, 150_deg}};
}  // namespace VisionConstants

namespace AutoConstants {
  inline constexpr auto kMaxSpeed = 3_mps;
  inline constexpr auto kMaxAcceleration = 3_mps_sq;
  inline constexpr auto kMaxAngularSpeed =
      units::radians_per_second_t(std::numbers::pi);
  inline constexpr auto kMaxAngularAcceleration =
      units::radians_per_second_squared_t(std::numbers::pi);

  inline constexpr pathplanner::PIDConstants kPIDTranslation{1.25, 0, 0.07};
  inline constexpr pathplanner::PIDConstants kPIDRotation{1, 0, 0.1};
  inline static pathplanner::RobotConfig kConfig = pathplanner::RobotConfig::fromGUISettings();

/*
  inline constexpr pathplanner::HolonomicPathFollowerConfig kConfig{
      kPIDTranslation, kPIDRotation, kMaxSpeed,
      0.53881_m, /**std::sqrt(2 * 15in ^ 2)**//*
      pathplanner::ReplanningConfig()};
*/

  inline constexpr pathplanner::PathConstraints kConstraints{
      kMaxSpeed, kMaxAcceleration, kMaxAngularSpeed, kMaxAngularAcceleration};
} // namespace AutoConstants

namespace OIConstants {
inline constexpr int kDriverControllerPort = 0;
inline constexpr int kOperatorControllerPort = 1;
}  // namespace OIConstants


namespace ClawConstants {
  inline constexpr int kIntakeMotorPort = 22;
  inline constexpr int kPivotMotorPort = 46;
  inline constexpr int kSensorPort = 14;

  inline constexpr float kPivotEncoderRatio = 1.0;

  inline constexpr double kP = 0.05000000298023224;
  inline constexpr double kI = 0;
  inline constexpr double kD = 0.30;

  inline constexpr units::second_t kSensorDelay = 0.18_s;
  
  namespace Speeds{
    inline constexpr double kStopped = 0;
    inline constexpr double kIntake = -.2;
    inline constexpr double kEject = .2;
  }
  namespace PivotPositions{
    inline constexpr double kIntake = 0.5; 
    inline constexpr double kL2 = 2.5;
    inline constexpr double kL3 = 3; 
    inline constexpr double kL4 = 4;
    inline constexpr double kCoral = 6; // left
    
    inline constexpr double kTollerance = .05;
  }
  
}

namespace ElevatorConstants {
  inline constexpr int kElevatorMotorPort = 10;
  inline constexpr int kElevatorMotor2Port = 11;
  inline constexpr double kElevatorEncoderRatio = 1;
  inline constexpr double kP = 0.2; 
  inline constexpr double kI = 0.0;
  inline constexpr double kD = 0.10999999940395355;
  inline constexpr int kTopLimitChannel = 0;
  inline constexpr int kBottomLimitChannel = 0;
  
  namespace Positions{
      inline constexpr double kL1 = 50;
      inline constexpr double kL3 = 200;
      inline constexpr double kL2 = 100;
      inline constexpr double kL4 = 250;
      inline constexpr double kIntake = 0;
      inline constexpr double kTolerance = 1;
  } // namespace Positions
}  // namespace ElevatorConstants

namespace AutoAlignConstants{
  inline constexpr double kXP = 0.125;
  inline constexpr double kXI = 0;
  inline constexpr double kXD = 0.01;

  inline constexpr double kYP = 0.125;
  inline constexpr double kYI = 0;
  inline constexpr double kYD = 0.01;

  inline constexpr double kRotP = 0.125;
  inline constexpr double kRotI = 0;
  inline constexpr double kRotD = 0.01;

  inline constexpr double kRotSetpoint = 0;
  inline constexpr double kXSetpoint = 15;
  inline constexpr double kRightYSetpoint = 19.34;
  inline constexpr double kLeftYSetPoint = -17.4;  

  inline constexpr double kRotTolerance = 2;
  inline constexpr double kXTolerance = 0.01;
  inline constexpr double kYTolerance = 0.01;

}