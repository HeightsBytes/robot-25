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
  m_lastRobotAngle = 0_deg;
  m_targetAngle = 0_deg;
  m_hasTarget = false;

  m_xController.EnableContinuousInput(-180, 180);
  m_yController.EnableContinuousInput(-180, 180);
  m_rotController.EnableContinuousInput(-180, 180);
}

// Called when the command is initially scheduled.
void AutoAlign::Initialize() {



}

// Called repeatedly when this Command is scheduled to run
void AutoAlign::Execute() {

    // PID
  m_rotController.SetSetpoint(AutoAlignConstants::kRotSetpoint);
  m_rotController.SetTolerance(AutoAlignConstants::kRotTolerance);

  m_xController.SetSetpoint(AutoAlignConstants::kXSetpoint);
  m_xController.SetTolerance(AutoAlignConstants::kXTolerance);

  m_yController.SetSetpoint(kIsRight ? AutoAlignConstants::kRightYSetpoint : AutoAlignConstants::kLeftYSetPoint);
  m_yController.SetTolerance(AutoAlignConstants::kYTolerance);


  frc::SmartDashboard::PutBoolean("has target", m_hasTarget);
  frc::SmartDashboard::PutBoolean("AutoAlign Finished", false);
  
  m_result = m_camera.GetLatestResult();
  m_hasTarget = m_result.HasTargets();
  m_target = m_result.GetBestTarget();

  m_targetAngle = units::degree_t(m_target.GetYaw());
  m_lastRobotAngle = m_drive->GetHeading().Degrees();

  units::meters_per_second_t xSpeed = units::meters_per_second_t(m_xController.Calculate(m_target.GetArea()) / 16);
  units::meters_per_second_t ySpeed = units::meters_per_second_t(m_yController.Calculate(m_target.GetYaw()) / 16);
  units::radians_per_second_t rotValue = units::radians_per_second_t(m_rotController.Calculate(m_target.GetSkew()) / 16);

  m_drive->Drive(xSpeed, ySpeed, rotValue, false);
}

// Called once the command ends or is interrupted.
void AutoAlign::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoAlign::IsFinished() {
  if(!m_hasTarget){
    frc::SmartDashboard::PutBoolean("AutoAlign Finished", true);
    return true;
  } else

  if(m_rotController.AtSetpoint() || m_xController.AtSetpoint() || m_yController.AtSetpoint()){
    frc::Wait(2_s);
    frc::SmartDashboard::PutBoolean("AutoAlign Finished", true);
    return true;
  } else {
    frc::SmartDashboard::PutBoolean("AutoAlign Finished", false);
    return false;
  }
}
