// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoAlign.h"
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

AutoAlign::AutoAlign(DriveSubsystem* drive, bool kIsRight) :
  m_drive(drive),
  m_camera("aprilcamera"),
  kIsRight(kIsRight)
{
  AddRequirements(drive);
  m_hasTarget = false;
}

// Called when the command is initially scheduled.
void AutoAlign::Initialize() {

    // PID
  m_controller.SetTolerance(AutoAlignConstants::kTolerance);

  m_targetPose = frc::Pose2d(
    frc::Translation2d(units::meter_t(AutoAlignConstants::kXSetpoint), units::meter_t(kIsRight ? AutoAlignConstants::kRightYSetpoint : AutoAlignConstants::kLeftYSetPoint)),
    frc::Rotation2d(units::radian_t(AutoAlignConstants::kRotSetpoint))
  );

}

// Called repeatedly when this Command is scheduled to run
void AutoAlign::Execute() {

  frc::SmartDashboard::PutBoolean("has target", m_hasTarget);
  frc::SmartDashboard::PutBoolean("AutoAlign Finished", false);
  
  // set camera stuff
  m_result = m_camera.GetLatestResult();
  m_hasTarget = m_result.HasTargets();
  m_target = m_result.GetBestTarget();
  m_bestCameraToTarget = m_target.GetBestCameraToTarget();

  m_robotPose = frc::Pose2d(
    frc::Translation2d(m_bestCameraToTarget.X(), m_bestCameraToTarget.Y()),
    frc::Rotation2d(m_bestCameraToTarget.Rotation().Angle())
  );

  frc::ChassisSpeeds speeds = m_controller.Calculate(m_robotPose, m_targetPose, units::meters_per_second_t(0.01), m_targetPose.Rotation());
  units::meters_per_second_t xSpeed = speeds.vx;
  units::meters_per_second_t ySpeed = speeds.vy;
  units::radians_per_second_t rotValue = speeds.omega;

  frc::SmartDashboard::PutNumber("xSpeed", xSpeed.value());
  frc::SmartDashboard::PutNumber("ySpeed", ySpeed.value());
  frc::SmartDashboard::PutNumber("rotValue", rotValue.value());

  //m_drive->Drive(xSpeed, ySpeed, rotValue, false);
}

// Called once the command ends or is interrupted.
void AutoAlign::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoAlign::IsFinished() {
  if(!m_hasTarget){
    frc::SmartDashboard::PutBoolean("AutoAlign Finished", false);
    return true;
  } else

  if(m_controller.AtReference()){
    frc::SmartDashboard::PutBoolean("AutoAlign Finished", true);
    return true;
  } else {
    frc::SmartDashboard::PutBoolean("AutoAlign Finished", false);
    return false;
  }
}
