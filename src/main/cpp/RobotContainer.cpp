// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <cameraserver/CameraServer.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/POVButton.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>

#include "commands/Intake.h"

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
  m_chooser.SetDefaultOption("None", "None");
  m_chooser.AddOption("leave", "leave");
  m_chooser.AddOption("1C-L3", "1C-L3");

  // Other Commands
  pathplanner::NamedCommands::registerCommand(
      "drive_switch", std::move(m_drive.SetGyro(180_deg)));

  frc::SmartDashboard::PutData("PDP", &m_pdp);
  frc::SmartDashboard::PutData("Auto Chooser", &m_chooser);
  frc::SmartDashboard::PutData("Command Scheduler",
                               &frc2::CommandScheduler::GetInstance());
  //frc::SmartDashboard::PutBoolean("Operator Manual Mode", false);

  //frc::CameraServer::StartAutomaticCapture();

  // Configure the button bindings
  ConfigureDriverButtons();
  ConfigureOperatorButtons();

  // Uses right trigger + left stick axis + right stick axis
  m_drive.SetDefaultCommand(DefaultDrive(
      &m_drive, [this] { return m_driverController.GetLeftX(); },
      [this] { return m_driverController.GetLeftY(); },
      [this] { return m_driverController.GetRightX(); },
      [this] { return m_driverController.GetRightTriggerAxis(); }));

}


void RobotContainer::ConfigureDriverButtons() {
  m_driverController.A().OnTrue(frc2::cmd::Print("Example!"));
}

void RobotContainer::ConfigureOperatorButtons() {

    m_operatorController.RightTrigger().OnTrue(m_elevator.SetTargetCMD(ElevatorSubsystem::GetNextState(m_elevator.GetTarget())));
    m_operatorController.LeftTrigger().OnTrue(m_elevator.SetTargetCMD(ElevatorSubsystem::GetPreviousState(m_elevator.GetTarget())));

    frc2::POVButton(&m_operatorController.GetHID(), 0).OnTrue(m_claw.SetPivotTargetCMD(ClawSubsystem::PivotState::kCoral)); // up
    frc2::POVButton(&m_operatorController.GetHID(), 90).OnTrue(m_claw.SetPivotTargetCMD(ClawSubsystem::PivotState::kL2)); // right
    frc2::POVButton(&m_operatorController.GetHID(), 180).OnTrue(m_claw.SetPivotTargetCMD(ClawSubsystem::PivotState::kIntake)); // down
    frc2::POVButton(&m_operatorController.GetHID(), 270).OnTrue(m_claw.SetPivotTargetCMD(ClawSubsystem::PivotState::kL3)); // left

    m_operatorController.LeftBumper().OnTrue(m_claw.SetIntakeTargetCMD(ClawSubsystem::IntakeState::kStopped));
    m_operatorController.RightBumper().OnTrue(m_claw.SetIntakeTargetCMD(ClawSubsystem::IntakeState::kIntaking));

    m_operatorController.B().OnTrue(m_claw.SetIntakeTargetCMD(ClawSubsystem::IntakeState::kEjecting));
    m_operatorController.Y().OnTrue(m_elevator.SetTargetCMD(ElevatorSubsystem::ElevatorState::kL2));
    m_operatorController.X().OnTrue(
      Intake(&m_claw).ToPtr()
      );

    //m_operatorController.LeftTrigger().OnTrue(m_claw.Intake());
    //m_operatorController.LeftBumper().OnTrue(m_claw.SetIntakeTargetCMD(ClawSubsystem::IntakeState::kStopped));
  
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  auto selected = m_chooser.GetSelected();
  if (selected == "None") {
    return frc2::cmd::None();
  } else {
    return pathplanner::PathPlannerAuto(selected).ToPtr();
  }
}

