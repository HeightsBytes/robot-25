#pragma once

#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <pathplanner/lib/path/PathConstraints.h>
//#include <pathplanner/lib/config/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/config/PIDConstants.h>
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
        inline constexpr int kFrontLeftDriveMotorPort = 1; // placeholder
        inline constexpr int kRearLeftDriveMotorPort = 1;
        inline constexpr int kFrontRightDriveMotorPort = 1;
        inline constexpr int kRearRightDriveMotorPort = 1;

        inline constexpr int kFrontLeftTurningMotorPort = 1;
        inline constexpr int kRearLeftTurningMotorPort = 1;
        inline constexpr int kFrontRightTurningMotorPort = 1;
        inline constexpr int kRearRightTurningMotorPort = 1;

        inline constexpr int kFrontLeftTurningEncoderPorts = 1;
        inline constexpr int kRearLeftTurningEncoderPorts = 1;
        inline constexpr int kFrontRightTurningEncoderPorts = 1;
        inline constexpr int kRearRightTurningEncoderPorts = 1;

        inline constexpr int kPidgeonID = 0;
    } // namespace CanIds
  inline constexpr double kFrontRightOffset = 0;
  inline constexpr double kFrontLeftOffset = 0;
  inline constexpr double kRearRightOffset = 0;
  inline constexpr double kRearLeftOffset = 0;

  inline constexpr auto kMaxChassisSpeed = 4.25_mps;
  inline constexpr auto kMaxAngularSpeed =
    units::radians_per_second_t(1 * std::numbers::pi);
  inline constexpr auto kMaxAngularAcceleration =
    units::radians_per_second_squared_t(2 * std::numbers::pi);

  inline constexpr auto kTrackWidth = 0.31369_m;
  inline constexpr auto kTrackLength = 0.31369_m;

  inline frc::SwerveDriveKinematics<4> kDriveKinematics{
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
